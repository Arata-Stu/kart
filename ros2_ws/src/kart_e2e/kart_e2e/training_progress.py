"""Single-line terminal progress and recognizable GUI progress snapshots; no CUDA sync."""
import sys
import shutil
import time

PREFIX = '[progress] '


def duration(seconds):
    if seconds is None: return '--:--'
    seconds = max(0,int(seconds))
    return f'{seconds//60:02d}:{seconds%60:02d}'


class TrainingProgress:
    def __init__(self, epochs=None, *, stream=None, interval=.5, clock=time.monotonic):
        self.epochs = epochs
        self.stream = stream if stream is not None else sys.stdout
        self.tty = bool(getattr(self.stream,'isatty',lambda:False)())
        self.interval,self.clock = interval,clock
        self.previous = float('-inf')
        self.key = None
        self.width = 0

    def update(self, phase, epoch, current, total, elapsed, samples, *, loss=None, mse=None, wait_ms=None, force=False):
        key=(phase,epoch)
        now=self.clock()
        if not force and self.key==key and now-self.previous<self.interval:return
        self.key,self.previous=key,now
        fraction=min(1,current/max(total,1))
        filled=int(fraction*16)
        bar='#'*filled+'-'*(16-filled)
        epoch_label=f'Epoch {epoch}/{self.epochs or "?"} | ' if epoch is not None else ''
        rate=samples/elapsed if elapsed>0 else 0
        eta=elapsed*(total-current)/current if current else None
        text=f'{epoch_label}{phase} [{bar}] {current}/{total}'
        if loss is not None:text+=f' | loss={loss:.5f}'
        if mse is not None and not self.tty:text+=f' mse={mse:.5f}'
        text+=f' | {rate:.1f} samples/s | {duration(elapsed)} ETA {duration(eta)}'
        if wait_ms is not None:text+=f' | data wait={wait_ms:.1f}ms'
        if self.tty:
            text=text[:max(20,shutil.get_terminal_size(fallback=(120,24)).columns-1)]
            self.stream.write('\r'+text+' '*max(0,self.width-len(text)))
            self.width=len(text)
        else:
            self.stream.write(PREFIX+text+'\n')
        self.stream.flush()

    def clear(self):
        if self.tty and self.width:
            self.stream.write('\r'+' '*self.width+'\r');self.stream.flush()
            self.width=0

    def summary(self, epoch, metrics, *, elapsed, best=False):
        self.clear()
        text=(f'Epoch {epoch}/{self.epochs or "?"} | '
              f'train loss={metrics["train_loss"]:.5f} mse={metrics["train_mse"]:.5f} | '
              f'val loss={metrics["validation_loss"]:.5f} mse={metrics["validation_mse"]:.5f}')
        if 'train_samples_per_second' in metrics:
            text+=f' | train {metrics["train_samples_per_second"]:.1f} / val {metrics["validation_samples_per_second"]:.1f} samples/s'
        text+=f' | {duration(elapsed)}'
        if 'learning_rate' in metrics:text+=f' | lr={metrics["learning_rate"]:g}'
        if best:text+=' | best saved'
        self.stream.write(text+'\n');self.stream.flush()
