// Adapt the existing CLI profile fields without changing saved preset format.
export function stereoState(values, defaults) {
  let infra=values.infra_profile ?? defaults.infra_profile;
  if(values.infra_fps!=null && values.infra_fps!=='') {
    infra=String(values.infra_fps)==='0'?'0':defaults.infra_profile.split('x').slice(0,2).concat(String(values.infra_fps)).join('x');
  }
  const depth=values.depth_profile ?? defaults.depth_profile;
  const infraOn=String(infra)!=='0', depthOn=String(depth)!=='0';
  return {infraOn,depthOn,profile:infraOn?infra:depthOn?depth:defaults.profile,
    conflict:infraOn && depthOn && infra!==depth};
}
export function setStereo(values, stereo) {
  delete values.infra_fps;
  values.infra_profile=stereo.infraOn?stereo.profile:'0';
  values.depth_profile=stereo.depthOn?stereo.profile:'0';
}
