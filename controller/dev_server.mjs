/**
 * dev_server.mjs —— 零依赖极速开发服务器（利用 Node 24 内置 stripTypeScriptTypes 实时编译 .ts）
 * 用法：无需等待 npm install，直接运行 `node controller/dev_server.mjs` 即可在浏览器打开 http://localhost:5173
 */
import http from 'node:http';
import fs from 'node:fs';
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

server.listen(PORT, '0.0.0.0', () => {
  console.log(`\n[UAV Controller] 地面站与手柄前端已启动: http://localhost:${PORT}\n`);
});
