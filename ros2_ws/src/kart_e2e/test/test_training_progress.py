"""Readable bounded-rate progress; terminal overwrite and one permanent epoch result."""
import io
from unittest.mock import patch

from kart_e2e.training_progress import TrainingProgress


def test_redirected_progress_throttling_and_epoch_summary():
    output=io.StringIO();now=[0.]
    progress=TrainingProgress(10,stream=output,clock=lambda:now[0])
    progress.update('train',1,0,10,0,0)
    now[0]=.1
    progress.update('train',1,1,10,1,32,loss=.1)
    assert len(output.getvalue().splitlines())==1
    now[0]=1.
    progress.update('train',1,5,10,2,160,loss=.1,mse=.02,wait_ms=3)
    assert 'Epoch 1/10 | train' in output.getvalue()
    assert 'loss=0.10000 mse=0.02000' in output.getvalue()
    assert '80.0 samples/s' in output.getvalue() and 'ETA 00:02' in output.getvalue()
    metrics=dict(train_loss=.1,train_mse=.02,validation_loss=.2,validation_mse=.03,
                 train_samples_per_second=80.,validation_samples_per_second=90.,learning_rate=1e-4)
    progress.summary(1,metrics,elapsed=4.,best=True)
    lines=output.getvalue().splitlines()
    assert len(lines)==3 and lines[-1].startswith('Epoch 1/10 | train loss=')
    assert 'best saved' in lines[-1] and 'lr=0.0001' in lines[-1]


def test_terminal_updates_overwrite_and_fit_width():
    class Terminal(io.StringIO):
        def isatty(self):return True
    output=Terminal();progress=TrainingProgress(10,stream=output)
    with patch('kart_e2e.training_progress.shutil.get_terminal_size',return_value=__import__('os').terminal_size((80,24))):
        progress.update('train',1,1,10,1,32,loss=.1,force=True)
        progress.update('train',1,2,10,2,64,loss=.05,force=True)
    assert '\n' not in output.getvalue() and '[progress]' not in output.getvalue()
    assert all(len(part)<=79 for part in output.getvalue().split('\r'))
    assert 'loss=0.05000' in output.getvalue()
    progress.clear();assert progress.width==0
