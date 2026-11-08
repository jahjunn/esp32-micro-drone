/**
 * dev_server.mjs —— 零依赖极速开发服务器（利用 Node 24 内置 stripTypeScriptTypes 实时编译 .ts）
 * 用法：无需等待 npm install，直接运行 `node controller/dev_server.mjs` 即可在浏览器打开 http://localhost:5173
 */
import http from 'node:http';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { stripTypeScriptTypes } from 'node:module';
import { fileURLToPath } from 'node:url';

const ROOT_DIR = path.dirname(fileURLToPath(import.meta.url));
const PORT = Number(process.env.PORT || 5173);

const MIME_MAP = {
  '.html': 'text/html; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.js': 'application/javascript; charset=utf-8',
  '.mjs': 'application/javascript; charset=utf-8',
  '.ts': 'application/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
};

const server = http.createServer((req, res) => {
  const urlPath = (req.url || '/').split('?')[0];
  let relPath = urlPath === '/' ? '/index.html' : urlPath;

  let filePath = path.join(ROOT_DIR, relPath);
  if (!fs.existsSync(filePath) && !path.extname(filePath)) {
    if (fs.existsSync(`${filePath}.ts`)) {
      filePath = `${filePath}.ts`;
    } else if (fs.existsSync(`${filePath}.js`)) {
      filePath = `${filePath}.js`;
    }
  }

  if (!filePath.startsWith(ROOT_DIR) || !fs.existsSync(filePath) || fs.statSync(filePath).isDirectory()) {
    res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
    res.end('404 Not Found');
    return;
  }

  const ext = path.extname(filePath);
  const contentType = MIME_MAP[ext] || 'application/octet-stream';

  try {
    if (ext === '.ts') {
      const tsSource = fs.readFileSync(filePath, 'utf8');
      const jsOutput = stripTypeScriptTypes(tsSource, { mode: 'strip' });
      res.writeHead(200, { 'Content-Type': contentType, 'Cache-Control': 'no-store' });
      res.end(jsOutput);
    } else {
      const content = fs.readFileSync(filePath);
      res.writeHead(200, { 'Content-Type': contentType, 'Cache-Control': 'no-store' });
      res.end(content);
    }
  } catch (err) {
    res.writeHead(500, { 'Content-Type': 'text/plain; charset=utf-8' });
    res.end(`Server Error: ${err.message}`);
  }
});

server.on('error', (err) => {
  if (err && err.code === 'EADDRINUSE') {
    console.error(`\n[错误] 端口 ${PORT} 已被占用（多半是上一次启动的服务还没关）。`);
    console.error(`  方案 1：关掉之前运行 dev_server 的终端（或在其中按 Ctrl+C）后重试；`);
    console.error(`  方案 2：换个端口：  $env:PORT=5174; node controller/dev_server.mjs\n`);
  } else if (err && err.code === 'EACCES') {
    console.error(`\n[错误] 没有权限监听端口 ${PORT}，请换一个端口（如 $env:PORT=5174）。\n`);
  } else {
    console.error('\n[错误] 开发服务器启动失败：', err, '\n');
  }
  process.exit(1);
});

server.listen(PORT, '0.0.0.0', () => {
  console.log(`\n[UAV Controller] 地面站与手柄前端已启动`);
  console.log(`  本机访问：  http://localhost:${PORT}`);

  // 列出局域网地址，并过滤掉 VPN / 代理软件(Clash 等)的虚拟网卡（198.18.x.x）与链路本地地址
  const lanUrls = [];
  const virtualUrls = [];
  for (const [name, addrs] of Object.entries(os.networkInterfaces())) {
    for (const a of addrs || []) {
      if (a.family !== 'IPv4' || a.internal) continue;
      const url = `http://${a.address}:${PORT}   (${name})`;
      if (a.address.startsWith('198.18.') || a.address.startsWith('198.19.') || a.address.startsWith('169.254.')) {
        virtualUrls.push(url);
      } else {
        lanUrls.push(url);
      }
    }
  }
  if (lanUrls.length > 0) {
    console.log('  手机(同一 Wi-Fi)请访问下面其中一个：');
    for (const u of lanUrls) console.log(`    ${u}`);
  } else {
    console.log('  [提示] 没找到可用的局域网地址，请确认电脑已连接 Wi-Fi / 网线。');
  }
  if (virtualUrls.length > 0) {
    console.log('  以下是虚拟网卡地址（VPN/代理产生），手机无法访问，请忽略：');
    for (const u of virtualUrls) console.log(`    ${u}`);
  }
  console.log('');
});
