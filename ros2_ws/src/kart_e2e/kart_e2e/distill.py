"""Stage 1: frozen RGB ViT-S CLS teacher -> trainable DINOv3 Tiny + linear projector.

Conceptual reference: uzh-rpg/generative_event_pretraining src/gra.py dimension
projection and MSE/cosine alignment; independently implemented for same-RGB compression.
"""
import argparse
import json
import math
from pathlib import Path
import random

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset

from .contract import SPEC, rgb_spec
from .feature_cache import cached_samples, digest
from .model import Policy, backbone, load_checkpoint
from .train import Samples


class AlignedSamples(Dataset):
    def __init__(self, images, features):
        self.images,self.features=images,features
    def __len__(self): return len(self.images)
    def __getitem__(self,index):
        image,_=self.images[index]
        feature,_=self.features[index]
        return image,feature


def alignment_loss(prediction, target, mse_weight=1., cosine_weight=.5):
    mse=nn.functional.mse_loss(prediction,target)
    cosine=nn.functional.cosine_similarity(prediction,target,dim=-1).clamp(-1,1).mean()
    return mse_weight*mse+cosine_weight*(1-cosine),mse,cosine


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--train',type=Path,nargs='+',required=True)
    p.add_argument('--validation',type=Path,nargs='+',required=True)
    p.add_argument('--teacher-weights',type=Path,required=True,help='Local official ViT-S state dict or kart RGB ViT-S best.pt')
    p.add_argument('--repo',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--device',default='cuda')
    p.add_argument('--epochs',type=int,default=10)
    p.add_argument('--batch-size',type=int,default=32)
    p.add_argument('--num-workers',type=int,default=8)
    p.add_argument('--learning-rate',type=float,default=1e-4)
    p.add_argument('--mse-weight',type=float,default=1.)
    p.add_argument('--cosine-weight',type=float,default=.5)
    p.add_argument('--feature-cache',action=argparse.BooleanOptionalAction,default=True)
    p.add_argument('--cache-dir',type=Path)
    p.add_argument('--seed',type=int,default=42)
    a=p.parse_args()
    if (a.epochs<1 or a.batch_size<1 or a.num_workers<0 or
            not math.isfinite(a.learning_rate) or a.learning_rate<=0 or
            any(not math.isfinite(x) or x<0 for x in (a.mse_weight,a.cosine_weight)) or
            a.mse_weight+a.cosine_weight==0):
        p.error('Invalid training settings')
    if a.output.exists(): raise FileExistsError(a.output)
    random.seed(a.seed);np.random.seed(a.seed);torch.manual_seed(a.seed)
    teacher_sha=digest(a.teacher_weights)
    state=torch.load(a.teacher_weights,map_location='cpu',weights_only=True)
    if 'spec' in state:
        if state['spec']!=SPEC: raise ValueError('Teacher must be an RGB ViT-S checkpoint')
        teacher=load_checkpoint(a.teacher_weights,a.repo,a.device)
    else:
        encoder=backbone(a.repo);encoder.load_state_dict(state,strict=True)
        teacher=Policy(encoder).to(a.device)
    del state
    teacher.eval().requires_grad_(False)
    if digest(a.teacher_weights)!=teacher_sha: raise ValueError('Teacher changed while loading')
    student=backbone(a.repo,'dinov3_vitt16').to(a.device)
    projector=nn.Linear(192,384).to(a.device)
    optimizer=torch.optim.AdamW([*student.parameters(),*projector.parameters()],lr=a.learning_rate)
    datasets=[Samples(roots,'steer_only') for roots in (a.train,a.validation)]

    def loader_factory(data,shuffle,persistent=True,seed=a.seed):
        settings=dict(batch_size=a.batch_size,shuffle=shuffle,num_workers=a.num_workers,
                      pin_memory=str(a.device).startswith('cuda'),generator=torch.Generator().manual_seed(seed))
        if a.num_workers: settings.update(multiprocessing_context='spawn',persistent_workers=persistent)
        return DataLoader(data,**settings)

    keys=[]
    if a.feature_cache:
        for i,data in enumerate(datasets):
            features,key=cached_samples(data,teacher,a.cache_dir or a.output.parent/'.feature_cache',
                                        teacher_sha,loader_factory,a.device)
            datasets[i]=AlignedSamples(data,features);keys.append(key)
    loaders=[loader_factory(data,i==0,seed=a.seed+i) for i,data in enumerate(datasets)]
    a.output.mkdir(parents=True)
    provenance={k:([str(p.resolve()) for p in v] if isinstance(v,list) else str(v) if isinstance(v,Path) else v)
                for k,v in vars(a).items()}
    provenance.update(teacher_sha256=teacher_sha,feature_cache_keys=keys,feature_target='x_norm_clstoken',
                      selection_metric='validation_loss',student_parameters=sum(v.numel() for v in student.parameters()),
                      architecture='dinov3_vitt16',projection='train_only_linear_192_to_384')
    (a.output/'run.json').write_text(json.dumps(provenance,indent=2))
    print(f'Distillation: teacher frozen; student={provenance["student_parameters"]} params; target=CLS; cache={a.feature_cache}',flush=True)
    best=float('inf')
    for epoch in range(a.epochs):
        metrics=dict(epoch=epoch+1)
        for i,loader in enumerate(loaders):
            student.train(i==0);projector.train(i==0)
            total=np.zeros(3,dtype=np.float64)
            with torch.set_grad_enabled(i==0):
                for images,target in loader:
                    images=images.to(a.device,non_blocking=True)
                    if a.feature_cache:
                        target=target.to(a.device,non_blocking=True)
                    else:
                        with torch.no_grad(): target=teacher.extract_features(images)
                    prediction=projector(student(images))
                    loss,mse,cosine=alignment_loss(prediction,target,a.mse_weight,a.cosine_weight)
                    if not torch.isfinite(loss): raise RuntimeError('Non-finite distillation loss')
                    if i==0:
                        optimizer.zero_grad();loss.backward();optimizer.step()
                    total+=np.array([loss.item(),mse.item(),cosine.item()])*len(images)
            prefix='train' if i==0 else 'validation'
            metrics.update({prefix+'_'+key:float(value/len(loader.dataset)) for key,value in zip(('loss','mse','cosine'),total)})
        with (a.output/'metrics.jsonl').open('a') as stream:stream.write(json.dumps(metrics)+'\n')
        print(metrics,flush=True)
        if metrics['validation_loss']<best:
            best=metrics['validation_loss']
            checkpoint=dict(format='kart.rgb_encoder.v1',spec=rgb_spec('dinov3_vitt16'),
                            encoder_state_dict=student.state_dict(),projection_state_dict=projector.state_dict(),
                            metrics=metrics,provenance=provenance)
            temporary=a.output/'encoder.tmp';torch.save(checkpoint,temporary);temporary.replace(a.output/'encoder.pt')


if __name__=='__main__':main()
