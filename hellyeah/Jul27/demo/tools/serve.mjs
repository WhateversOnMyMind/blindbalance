import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
const files = { '/': ['index.html','text/html'], '/style.css': ['style.css','text/css'],
  '/motion.js': ['motion.js','text/javascript'], '/renderer.js': ['renderer.js','text/javascript'], '/serial.js': ['serial.js','text/javascript'], '/app.js': ['app.js','text/javascript'] };
const server = createServer(async (req,res) => {
  const path = new URL(req.url, 'http://localhost').pathname;
  if (path === '/samples') {
    res.writeHead(503, {'Content-Type':'application/json'});
    res.end(JSON.stringify({error:'For a wired board, choose Connect USB. Wi-Fi mode uses the page hosted at http://192.168.4.1/ with DEMO_WIFI=1.'}));
    return;
  }
  const file = files[path];
  if (!file) { res.writeHead(404); res.end('Not found'); return; }
  try {
    const body = await readFile(new URL(`../web/${file[0]}`, import.meta.url));
    res.writeHead(200, {'Content-Type':file[1], 'Cache-Control':'no-store'});
    res.end(body);
  } catch { res.writeHead(500); res.end('Cannot read demo asset'); }
});
server.listen(Number(process.env.PORT || 8127), '127.0.0.1', () => console.log(`Preview: http://127.0.0.1:${server.address().port}/`));
