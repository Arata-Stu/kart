import importlib.util
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch

import pytest

LIB=Path(__file__).resolve().parents[1]/'lib'
sys.path.insert(0,str(LIB))
spec=importlib.util.spec_from_file_location('model_compare',LIB/'e2e_model_compare.py')
compare=importlib.util.module_from_spec(spec);spec.loader.exec_module(compare)


def fixture(tmp_path,extra=(),*,preset='full',pilot=2):
    data=tmp_path/'data';data.mkdir();(data/'image.npy').write_bytes(b'image')
    (data/'samples.jsonl').write_text('{"image":"image.npy","steering":0.1,"throttle":0.1}\n')
    (data/'metadata.json').write_text('{"modality":"rgb"}')
    repo=tmp_path/'repo';(repo/'dinov3').mkdir(parents=True)
    small=tmp_path/'small.pt';small.write_bytes(b'fixture')
    base=tmp_path/'base.pt';base.write_bytes(b'fixture-base')
    root=tmp_path/'comparison'
    argv=['compare','--train',str(data),'--repo',str(repo),'--weights',str(small),
          '--base-weights',str(base),'--output',str(root),'--dry-run']
    if preset is not None:argv+=['--preset',preset]
    if pilot is not None:argv+=['--pilot-epochs',str(pilot)]
    argv+=extra
    # Fixture need not be an actual DINOv3 Git checkout.
    with patch.object(sys,'argv',argv),patch.object(compare.subprocess,'check_output',return_value=b'fixture-git'):
        compare.main()
    return root,json.loads((root/'plan.json').read_text())


def fake_run(folder,args,deadline):
    folder.mkdir(parents=True,exist_ok=True);output=folder/'output';output.mkdir(exist_ok=True)
    distill='kart_e2e.distill' in args
    count=int(args[args.index('--epochs')+1])
    (output/('encoder.pt' if distill else 'best.pt')).write_bytes(b'fixture')
    (output/'metrics.jsonl').write_text('\n'.join(json.dumps(dict(epoch=i+1,
        train_seconds=1,validation_seconds=.5,validation_loss=1/(i+1),
        validation_mse=.1/(i+1),validation_mae=[.01])) for i in range(count))+'\n')
    compare.save(output/'run.json',dict(total_parameters=100,trainable_parameters=10,effective_precision='fp32'))
    compare.save(folder/'command.json',args)
    compare.save(folder/'status.json',dict(status='succeeded',wall_seconds=count*1.5+2))
    return True


def test_plan_recipe_distillation_dependencies_summary_and_resume(tmp_path):
    root,plan=fixture(tmp_path)
    assert plan['hours']==6 and plan['epochs']==100 and len(plan['cases'])==11
    assert plan['same_train_validation']
    calls=[]
    def run(*args):calls.append(args[1]);return fake_run(*args)
    with patch.object(compare.subprocess,'check_output',return_value=b'fixture-git'),patch.object(compare,'run',run):
        compare.execute(root,plan)
        count=len(calls);compare.execute(root,plan)
    assert len(calls)==count
    summaries=json.loads((root/'summary.json').read_text())['results']
    assert all(r['status']=='succeeded' for r in summaries)
    for n in (10,50,100):
        row=next(r for r in summaries if r['name']==f'tiny_distill_{n}')
        assert row['encoder_best_epoch']==n and row['completed_epochs']==100
        folder=root/'runs'/f'tiny_distill_{n}'
        args=json.loads((folder/'command.json').read_text())
        assert str(root/'runs'/f'tiny_distill_{n}_encoder/output/encoder.pt') in args
        assert '--finetune' not in args and '--feature-cache' in args
        assert row['preparation_seconds']==(n*1.5+2)+(100*1.5+2)
    for args in calls:
        if 'kart_e2e.train' in args:
            assert args[args.index('--lr-scheduler')+1]=='plateau'
            assert args[args.index('--loss')+1]=='smooth_l1'
    scratch=next(args for args in calls if '--initialization' in args and args[args.index('--initialization')+1]=='scratch')
    assert '--weights' not in scratch and '--no-feature-cache' in scratch
    assert 'budget_epochs' in (root/'epoch_budgets.csv').read_text()
    assert len(list((root/'pilots').glob('distill')))==1
    with patch.object(compare.subprocess,'check_output',return_value=b'fixture-git'):
        Path(plan['small_weights']).write_bytes(b'changed')
        with pytest.raises(ValueError,match='changed'):compare.execute(root,plan)


def test_budget_skips_whole_distillation_chain_without_lowering_epochs(tmp_path):
    root,plan=fixture(tmp_path,('--cases','tiny_distill_100'))
    calls=[]
    def run(*args):calls.append(args[1]);return fake_run(*args)
    with patch.object(compare.subprocess,'check_output',return_value=b'fixture-git'),patch.object(compare,'run',run):
        compare.execute(root,plan,hours=.01)
    assert len(calls)==2 and all(args[args.index('--epochs')+1]=='2' for args in calls)
    assert not (root/'runs').exists()
    row=compare.status(root/'cases/tiny_distill_100')
    assert row['status']=='budget_skipped' and row['estimate_seconds']>row['remaining_seconds']
    # A new invocation has a fresh time budget; completed timing pilots are reused.
    with patch.object(compare.subprocess,'check_output',return_value=b'fixture-git'),patch.object(compare,'run',fake_run):
        compare.execute(root,plan,hours=1)
    assert compare.status(root/'cases/tiny_distill_100')['status']=='succeeded'


def test_size_preset_shares_encoder_and_compares_matching_scopes(tmp_path):
    root,plan=fixture(tmp_path,preset=None)
    assert len(plan['cases'])==6 and plan['preset']=='sizes'
    assert not any(c['initialization']=='scratch' for c in plan['cases'])
    calls=[]
    def run(folder,args,deadline):
        calls.append((folder,args))
        return fake_run(folder,args,deadline)
    with patch.object(compare.subprocess,'check_output',return_value=b'fixture-git'),patch.object(compare,'run',run):
        compare.execute(root,plan)
    encoders=[folder for folder,args in calls if 'kart_e2e.distill' in args and folder.parent.name=='runs']
    assert [f.name for f in encoders]==['tiny_distill_50_encoder']
    frozen=json.loads((root/'runs/tiny_distill_50/command.json').read_text())
    fine=json.loads((root/'runs/tiny_distill_50_finetune/command.json').read_text())
    assert frozen[frozen.index('--weights')+1]==fine[fine.index('--weights')+1]
    assert '--finetune' not in frozen and '--feature-cache' in frozen
    assert '--finetune' in fine and '--no-feature-cache' in fine
    assert fine[fine.index('--learning-rate')+1]=='1e-05'
    assert (root/'pilots/tiny_distilled_finetune/status.json').exists()
    import csv
    rows=list(csv.DictReader((root/'size_comparison.csv').open()))
    for row in rows:
        assert row['reference']==('b_finetune' if row['scope']=='finetune' else 'b_freeze')
        assert float(row['mse_ratio_to_b'])==1
    assert 'not architecture alone' in (root/'size_comparison.md').read_text()
    # Incomplete B results cannot act as the comparison reference.
    compare.save(root/'cases/b_finetune/status.json',dict(status='budget_timeout'))
    compare.summarize(root,plan)
    rows=list(csv.DictReader((root/'size_comparison.csv').open()))
    assert all(not row['reference'] for row in rows if row['scope']=='finetune')


def test_default_runs_100_epochs_directly_without_pilots(tmp_path):
    root,plan=fixture(tmp_path,preset=None,pilot=None)
    assert plan['pilot_epochs']==0 and plan['epochs']==100
    calls=[]
    def run(folder,args,deadline):
        calls.append((folder,args));return fake_run(folder,args,deadline)
    with patch.object(compare.subprocess,'check_output',return_value=b'fixture-git'),patch.object(compare,'run',run):
        compare.execute(root,plan)
    assert not (root/'pilots').exists()
    assert len(calls)==7  # Six control runs and one shared encoder distillation.
    for folder,args in calls:
        assert folder.parent.name=='runs'
        assert args[args.index('--epochs')+1]==('50' if 'kart_e2e.distill' in args else '100')
    assert all(r['status']=='succeeded' and r['estimate_seconds'] is None for r in compare.summarize(root,plan))


def test_direct_training_does_not_start_after_budget_expires(tmp_path):
    root,plan=fixture(tmp_path,('--cases','s_freeze'),pilot=None)
    with patch.object(compare.subprocess,'check_output',return_value=b'fixture-git'),patch.object(compare,'run') as run:
        compare.execute(root,plan,hours=.00001)
    run.assert_not_called()
    assert compare.status(root/'cases/s_freeze')['status']=='budget_skipped'


def test_failed_pilot_continues_other_cases_and_missing_base_is_explicit(tmp_path):
    root,plan=fixture(tmp_path,('--cases','s_finetune','cnn_scratch','b_freeze'))
    Path(plan['base_weights']).unlink()
    with patch.object(compare.subprocess,'check_output',return_value=b'fixture-git'):
        plan['signatures']=compare.signatures(plan)
    def run(folder,args,deadline):
        if folder.name=='s_finetune':
            folder.mkdir(parents=True);compare.save(folder/'status.json',dict(status='oom'));return False
        return fake_run(folder,args,deadline)
    with patch.object(compare.subprocess,'check_output',return_value=b'fixture-git'),patch.object(compare,'run',run):
        compare.execute(root,plan)
    rows={r['name']:r for r in compare.summarize(root,plan)}
    assert rows['s_finetune']['status']=='oom'
    assert rows['b_freeze']['status']=='unavailable'
    assert rows['cnn_scratch']['status']=='succeeded'


def test_deadline_stops_process_and_preserves_outputs(tmp_path):
    folder=tmp_path/'run';(folder/'output').mkdir(parents=True)
    (folder/'output/partial').write_text('preserve')
    assert not compare.run(folder,[sys.executable,'-c','import time; time.sleep(20)'],time.monotonic()+.1)
    assert compare.status(folder)['status']=='budget_timeout'
    assert list(folder.glob('interrupted-*/partial'))


@pytest.mark.parametrize('extra',[('--hours','nan'),('--hours','-1'),('--pilot-epochs','-1'),('--distill-epochs','0'),('--cases','unknown')])
def test_invalid_configuration(tmp_path,extra):
    with pytest.raises((SystemExit,ValueError)):fixture(tmp_path,extra)
