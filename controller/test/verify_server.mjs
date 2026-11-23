import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { once } from 'node:events';
import { mkdtemp, writeFile, rm, rmdir } from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
// 与 controller 共享路径前缀，确保路径边界检查不会放行相邻目录。
const sibling = await mkdtemp(`${root}-boundary-`);
const child = spawn(process.execPath, [path.join(root, 'dev_server.mjs')], {
  env: { ...process.env, PORT: '0' }, stdio: ['ignore', 'pipe', 'pipe'],
});
let startup = '';
let errors = '';
child.stderr.on('data', (data) => { errors += data; });
const request = (port, route) => new Promise((resolve, reject) => {
  http.get({ host: '127.0.0.1', port, path: route }, (res) => {
    let body = '';
    res.on('data', (data) => { body += data; });
    res.on('end', () => resolve({ status: res.statusCode, body }));
  }).on('error', reject);
});
try {
  await writeFile(path.join(sibling, 'probe.txt'), 'outside-root');
  const port = await new Promise((resolve, reject) => {
    const timeout = setTimeout(() => reject(new Error(`Server startup timed out: ${errors}`)), 5000);
    child.once('error', (error) => { clearTimeout(timeout); reject(error); });
    child.once('exit', (code) => { clearTimeout(timeout); reject(new Error(`Server exited ${code}: ${errors}`)); });
    child.stdout.on('data', (data) => {
      startup += data;
      const match = startup.match(/localhost:(\d+)/);
      if (match) { clearTimeout(timeout); resolve(Number(match[1])); }
    });
  });
  assert.equal((await request(port, '/')).status, 200);
  const main = await request(port, '/src/main.ts');
  assert.equal(main.status, 200);
  assert.ok(!main.body.includes('interface WakeLockSentinelLike'));
  assert.equal((await request(port, `/../${path.basename(sibling)}/probe.txt`)).status, 404);
  console.log('[PASS] Development server entry, TypeScript transform and directory boundary');
} finally {
  if (child.exitCode === null) {
    const stopped = once(child, 'exit');
    child.kill();
    await stopped;
  }
  await rm(path.join(sibling, 'probe.txt'), { force: true });
  await rmdir(sibling);
}
