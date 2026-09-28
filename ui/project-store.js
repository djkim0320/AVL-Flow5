// Browser-side persistence of a project: the IndexedDB draft, the server copy and the file download.

export function openDraftDB() {
  return new Promise((resolve, reject) => {
    const request = indexedDB.open('dbf-assembly-editor', 1);
    request.onupgradeneeded = () => request.result.createObjectStore('projects');
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error);
  });
}

export function loadDraft(db, key = 'draft') {
  return new Promise((resolve, reject) => {
    const request = db.transaction('projects').objectStore('projects').get(key);
    request.onsuccess = () => resolve(request.result ?? null);
    request.onerror = () => reject(request.error);
  });
}

export function saveDraft(db, value, key = 'draft') {
  return new Promise((resolve, reject) => {
    const tx = db.transaction('projects', 'readwrite');
    tx.objectStore('projects').put(value, key);
    tx.oncomplete = () => resolve();
    tx.onerror = () => reject(tx.error);
  });
}

export async function postProject(value) {
  const response = await fetch('/api/projects', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(value)
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error);
  return result;
}

export function downloadJSON(value, name) {
  const blob = new Blob([JSON.stringify(value, null, 2)], { type: 'application/json' }),
    url = URL.createObjectURL(blob),
    link = document.createElement('a');
  link.href = url;
  link.download = name;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 10000);
}
