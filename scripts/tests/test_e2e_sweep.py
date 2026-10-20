import importlib.util
import json
from pathlib import Path
import sys
from unittest.mock import patch

import pytest

spec=importlib.util.spec_from_file_location('kart_sweep',Path(__file__).resolve().parents[1]/'lib/e2e_sweep.py')
sweep=importlib.util.module_from_spec(spec);spec.loader.exec_module(sweep)


def fixture(tmp_path):
    data=tmp_path/'data';data.mkdir();(data/'image.npy').write_bytes(b'image')
    (data/'samples.jsonl').write_text('{"image":"image.npy","steering":0,"throttle":0}')
    repo=tmp_path/'repo';(repo/'dinov3').mkdir(parents=True)
    weights=tmp_path/'weights.pt';weights.write_bytes(b'fixture')
    root=tmp_path/'sweep'
    args=['sweep','--train',str(data),'--repo',str(repo),'--weights',str(weights),'--output',str(root),'--epochs','3','--batches','32','64','--losses','mse','--schedulers','none','--dry-run']
    with patch.object(sys,'argv',args):sweep.main()
    return root,json.loads((root/'plan.json').read_text())


def test_plan_commands_same_data_and_epoch_summary(tmp_path):
    root,plan=fixture(tmp_path)
    assert plan['train']==plan['validation'] and len(plan['cases'])==2
    command=sweep.command(plan,plan['cases'][0],root/'output')
    assert '--finetune' in command and '--ram-preload' in command and '--no-feature-cache' in command
    calls=[]
    class Process:
        def __init__(self,args,**kwargs):self.args=args;calls.append(args)
        def wait(self):
            folder=Path(self.args[self.args.index('--output')+1]);folder.mkdir()
            (folder/'best.pt').write_bytes(b'weights')
            values=[.3,.1,.2] if '--batch-size' in self.args and self.args[self.args.index('--batch-size')+1]=='32' else [.2,.15,.05]
            (folder/'metrics.jsonl').write_text('\n'.join(json.dumps(dict(epoch=i+1,validation_mse=value,validation_loss=value,validation_mae=[value],train_seconds=1,validation_seconds=.5,elapsed_seconds=(i+1)*2)) for i,value in enumerate(values)))
            return 0
    with patch.object(sweep.subprocess,'Popen',Process):
        sweep.execute(root,plan)
        sweep.execute(root,plan) # Completed conditions are not rerun.
    assert len(calls)==2
    result=json.loads((root/'summary.json').read_text())
    assert result['results'][0]['best_epoch']==2 and result['results'][1]['best_epoch']==3
    assert 'budget_epochs' in (root/'epoch_budgets.csv').read_text()
    assert '64' in (root/'summary.md').read_text()
    # Data changes cannot silently enter a resumed comparison.
    (Path(plan['train'][0])/'image.npy').write_bytes(b'changed image')
    with pytest.raises(ValueError,match='changed'):sweep.execute(root,plan)


def test_oom_continues_and_interrupted_directory_is_preserved(tmp_path):
    root,plan=fixture(tmp_path)
    folder=root/plan['cases'][0]['name'];(folder/'training').mkdir(parents=True)
    (folder/'training/partial.txt').write_text('keep')
    calls=[]
    class Process:
        def __init__(self,args,stdout,**kwargs):calls.append(args);stdout.write('torch.OutOfMemoryError: CUDA out of memory\n')
        def wait(self):return 1
    with patch.object(sweep.subprocess,'Popen',Process):sweep.execute(root,plan)
    assert len(calls)==2
    assert list(folder.glob('interrupted-*/partial.txt'))
    rows=json.loads((root/'summary.json').read_text())['results']
    assert all(row['status']=='oom' for row in rows)
