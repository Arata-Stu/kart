// Keep progress snapshots out of history; display only the latest active snapshot.
export function compactLog(text) {
  const history = [];
  let progress = '';
  for (const line of text.split('\n')) {
    if (line.startsWith('[progress] ')) progress = line.slice(11).trimEnd();
    else {
      // Epoch summary is the permanent result; the phase progress has finished.
      if (/^Epoch \d+\//.test(line)) progress = '';
      history.push(line);
    }
  }
  return { text: history.join('\n'), progress };
}
