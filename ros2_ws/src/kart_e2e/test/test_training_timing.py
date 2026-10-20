"""Host attribution and asynchronous CUDA sampling semantics, without a GPU."""
import json
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import torch
from kart_e2e.training_timing import TrainingTiming


def test_data_wait_and_compute_are_separate_and_final_batch_count_exact(tmp_path):
    clock=SimpleNamespace(value=0.)
    class Loader:
        def __iter__(self):
            clock.value+=.03
            return self
        def __next__(self):
            if clock.value>.1:raise StopIteration
            clock.value+=.01
            return torch.zeros(2,1),torch.zeros(2,1)
    with patch('kart_e2e.training_timing.time.perf_counter',side_effect=lambda:clock.value):
        timing=TrainingTiming(tmp_path,'cpu',log_interval=2)
        phase=timing.phase('train',1)
        for batch in phase.batches(Loader()):
            with phase.measure('encoder'):clock.value+=.02
        report=phase.report()
    assert report['batches']==3 and report['samples']==6
    assert report['host_ms']['data_wait']['mean']==pytest.approx(10.)
    assert report['host_ms']['data_wait']['count']==3
    assert report['host_ms']['iterator_start']['total']==pytest.approx(30.)
    assert report['host_ms']['encoder']['total']==pytest.approx(60.)
    assert report['wall_seconds']==pytest.approx(.12)
    assert report['samples_per_second']==pytest.approx(50)
    assert not report['sampled_cuda_ms'] and report['cuda_memory'] is None
    rows=[json.loads(line) for line in timing.path.read_text().splitlines()]
    assert [row['batches'] for row in rows if row['event']=='progress']==[1,2]


def test_cuda_events_are_capped_and_only_phase_end_synchronizes(tmp_path):
    events=[];synchronizations=[]
    class Event:
        def __init__(self,**kwargs):events.append(self)
        def record(self):pass
        def query(self):return True
        def elapsed_time(self,other):return 2.5
    cuda=dict(Event=Event,device=lambda *_:__import__('contextlib').nullcontext(),
              reset_peak_memory_stats=lambda *_:None,get_device_name=lambda *_:'MOCK',
              get_device_properties=lambda *_:SimpleNamespace(total_memory=1000),
              memory_allocated=lambda *_:100,memory_reserved=lambda *_:200,max_memory_allocated=lambda *_:300,
              synchronize=lambda device:synchronizations.append(str(device)))
    with patch.multiple(torch.cuda,**cuda):
        timing=TrainingTiming(tmp_path,'cuda:1',profile_batches=2,log_interval=1)
        phase=timing.phase('validation',1)
        for batch in phase.batches([(torch.zeros(1,1),None)]*4):
            with phase.measure('encoder'):pass
            assert not synchronizations
        report=phase.report()
    assert len(events)==4 and synchronizations==['cuda:1']
    assert report['sampled_cuda_ms']['encoder']['count']==2
    assert report['sampled_cuda_ms']['encoder']['mean']==2.5
    assert report['host_ms']['encoder']['count']==4
    assert report['cuda_memory']['peak_allocated_bytes']==300


def test_invalid_sampling_configuration(tmp_path):
    for limit,interval in [(-1,1),(1001,1),(1,0)]:
        with pytest.raises(ValueError):TrainingTiming(tmp_path,'cpu',profile_batches=limit,log_interval=interval)


@pytest.mark.skipif(not torch.cuda.is_available(),reason='real CUDA GPU required')
def test_real_cuda_sample(tmp_path):
    timing=TrainingTiming(tmp_path,'cuda',profile_batches=1)
    phase=timing.phase('gpu_smoke',1)
    for batch in phase.batches([(torch.ones(8,8),None)]):
        with phase.measure('transfer'):value=batch[0].to('cuda')
        with phase.measure('encoder'):value=value@value
    report=phase.report()
    assert report['sampled_cuda_ms']['encoder']['count']==1
    assert report['sampled_cuda_ms']['encoder']['mean']>=0
