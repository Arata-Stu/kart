"""Bounded CUDA event sampling and live host-stage diagnostics for offline training."""
from contextlib import contextmanager
import json
from pathlib import Path
import platform
import time

import numpy as np
import torch


def stats(values):
    array = np.asarray(values, dtype=np.float64)
    return dict(count=len(values), total=float(array.sum()), mean=float(array.mean()),
                median=float(np.median(array)), p95=float(np.percentile(array, 95)),
                maximum=float(array.max()), first=float(array[0]),
                mean_after_first=float(array[1:].mean()) if len(array)>1 else None) if len(values) else None


class TrainingTiming:
    def __init__(self, output, device, *, profile_batches=20, log_interval=50):
        self.path = Path(output) / 'timing.jsonl'
        self.device = torch.device(device)
        self.profile_batches, self.log_interval = profile_batches, log_interval
        if not 0 <= profile_batches <= 1000 or not 1 <= log_interval <= 100000:
            raise ValueError('profile-batches must be 0..1000; log-interval must be 1..100000')
        self.cuda = self.device.type == 'cuda'
        hardware = dict(device=str(self.device), torch=str(torch.__version__),
                        python=platform.python_version(), cuda_runtime=torch.version.cuda,
                        cpu_threads=torch.get_num_threads())
        if self.cuda:
            hardware.update(gpu=torch.cuda.get_device_name(self.device),
                            total_memory_bytes=torch.cuda.get_device_properties(self.device).total_memory)
        self.write(dict(event='configuration', profile_batches=profile_batches,
                        log_interval=log_interval, hardware=hardware,
                        cuda_sampling='first N batches per phase including startup; compare first with mean_after_first',
                        data_wait_scope='consumer wait for next batch; includes worker IO/preprocessing/queue/pinning; not worker CPU time or disk throughput',
                        scope='host stage time includes dispatch and waits; CUDA event time is sampled stream elapsed time, not kernel utilization; stages overlap across CPU/GPU and must not be added together'))

    def write(self, row):
        with self.path.open('a') as stream:
            stream.write(json.dumps(dict(schema='kart.training_timing.v1', time_unix_s=time.time(), **row), allow_nan=False) + '\n')

    def record(self, phase, seconds, **details):
        self.write(dict(event='operation', phase=phase, seconds=seconds, **details))
        print(f'TIMING {phase}: {seconds:.3f}s ' + json.dumps(details), flush=True)

    def phase(self, name, epoch=None):
        return Phase(self, name, epoch)


class Phase:
    def __init__(self, owner, name, epoch):
        self.owner, self.name, self.epoch = owner, name, epoch
        self.started = time.perf_counter()
        self.batches_seen = self.samples = 0
        self.host, self.events = {}, {}
        if owner.cuda:
            torch.cuda.reset_peak_memory_stats(owner.device)
        owner.write(dict(event='phase_start', phase=name, epoch=epoch))
        print(f'TIMING {name} epoch={epoch}: started', flush=True)

    def add(self, name, milliseconds):
        self.host.setdefault(name, []).append(milliseconds)

    @contextmanager
    def measure(self, name, *, gpu=True):
        events = None
        if gpu and self.owner.cuda and self.batches_seen < self.owner.profile_batches:
            with torch.cuda.device(self.owner.device):
                events = (torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True))
                events[0].record()
        started = time.perf_counter()
        try:
            yield
        finally:
            self.add(name, (time.perf_counter() - started) * 1000)
            if events:
                with torch.cuda.device(self.owner.device): events[1].record()
                self.events.setdefault(name, []).append(events)

    def batches(self, loader):
        with self.measure('iterator_start', gpu=False): iterator = iter(loader)
        while True:
            started = time.perf_counter()
            try: batch = next(iterator)
            except StopIteration:
                self.add("iterator_end", (time.perf_counter()-started)*1000)
                break
            self.add('data_wait', (time.perf_counter()-started)*1000)
            yield batch
            self.batches_seen += 1
            self.samples += len(batch[0])
            if self.batches_seen == 1 or self.batches_seen % self.owner.log_interval == 0:
                self.report('progress')

    def report(self, event='phase_end'):
        if event == 'phase_end' and self.owner.cuda:
            # Once at phase end, never synchronize between individual stages.
            torch.cuda.synchronize(self.owner.device)
        wall = time.perf_counter() - self.started
        gpu = {name: stats([start.elapsed_time(end) for start, end in pairs if end.query()])
               for name, pairs in self.events.items()}
        memory = dict(allocated_bytes=torch.cuda.memory_allocated(self.owner.device),
                      reserved_bytes=torch.cuda.memory_reserved(self.owner.device),
                      peak_allocated_bytes=torch.cuda.max_memory_allocated(self.owner.device)) if self.owner.cuda else None
        report = dict(event=event, phase=self.name, epoch=self.epoch,
                      batches=self.batches_seen, samples=self.samples, wall_seconds=wall,
                      samples_per_second=self.samples/wall if wall else None,
                      host_ms={name:stats(values) for name,values in self.host.items()},
                      sampled_cuda_ms=gpu, cuda_sample_limit=self.owner.profile_batches,
                      cuda_memory=memory)
        self.owner.write(report)
        host = report['host_ms']
        wait = host.get('data_wait', {}).get('mean', 0)
        gpu_text = ', '.join(f'{name}={value["mean"]:.2f}' for name,value in gpu.items() if value)
        print(f'TIMING {self.name} epoch={self.epoch} batch={self.batches_seen}: '
              f'{wall:.2f}s, {report["samples_per_second"]:.1f} samples/s; '
              f'data_wait={wait:.2f}ms; host stage mean ms=' +
              ', '.join(f'{name}={value["mean"]:.2f}' for name,value in host.items() if name!='data_wait') +
              (f'; sampled CUDA mean ms: {gpu_text}' if gpu_text else ''), flush=True)
        return report
