// Pipeline identity is independent of modality (RGB / EVS) and head count.
export function isMulti(row) {
  return row?.pipeline==='multi' || Boolean(row?.routing || row?.routing_sha256 || row?.section_heads?.length);
}
export function pipelineRows(catalog, kind, pipeline, {teacher=false}={}) {
  const rows=catalog[kind] || [];
  if (kind==='encoders')return rows;
  if (kind==='datasets' && teacher)return rows.filter(r=>!isMulti(r));
  return rows.filter(r=>isMulti(r)===(pipeline==='multi'));
}
export function pipelineSteps(pipeline) {
  return pipeline==='multi' ? ['sections','dataset','train','export','evaluate','push'] :
    ['dataset','train','export','evaluate','features','push'];
}
