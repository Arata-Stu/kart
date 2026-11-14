// Named snapshots are independent of the working custom line and of each other.
export function snapshotCustom(lane, id, name) {
  name = name.trim();
  if (!name || name.length > 64 || /[\x00-\x1f\x7f]/.test(name)) throw Error('版名は1〜64文字で入力してください');
  return structuredClone({id, name, custom: lane.custom, custom_speeds: lane.custom_speeds || [],
    custom_speed_mode: lane.custom_speed_mode || 'profile',
    ...(lane.lines.customline ? {line: lane.lines.customline} : {})});
}

export function loadCustomVersion(lane, id) {
  const version = lane.custom_versions?.find(v => v.id === id);
  if (!version) throw Error('Customの版を選択してください');
  lane.custom = structuredClone(version.custom);
  lane.custom_speeds = structuredClone(version.custom_speeds);
  lane.custom_speed_mode = version.custom_speed_mode;
  lane.custom_version = id;
  delete lane.lines.customline;
  if (version.line) lane.lines.customline = structuredClone(version.line);
  return version;
}

export function nextVersionId(lane) {
  let i = 1;
  while (lane.custom_versions?.some(v => v.id === `custom_${String(i).padStart(3, '0')}`)) i++;
  return `custom_${String(i).padStart(3, '0')}`;
}
