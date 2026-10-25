// Offline cache for the demo: network first (so a fresh push always wins while developing),
// cached copy when the network is gone (venue Wi-Fi down, airplane mode).

const CACHE = 'lane-guide-v3';
const SHELL = [
  './spike-ground.html',
  './vendor/opencv.js',
  './lane/ground.js',
  './lane/detector.js',
  './lane/tracker.js',
  './control/estimator.js',
  './control/controller.js',
  './control/protocol.js',
  './control/logger.js',
  './control/audio.js',
  './control/tuning.js',
  './control/clip.js',
];

self.addEventListener('install', (event) => {
  event.waitUntil(caches.open(CACHE).then((cache) => cache.addAll(SHELL)));
  self.skipWaiting();
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((keys) => Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k)))),
  );
  self.clients.claim();
});

self.addEventListener('fetch', (event) => {
  if (event.request.method !== 'GET') return;
  event.respondWith(
    fetch(event.request)
      .then((response) => {
        const copy = response.clone();
        caches.open(CACHE).then((cache) => cache.put(event.request, copy));
        return response;
      })
      .catch(() => caches.match(event.request)),
  );
});
