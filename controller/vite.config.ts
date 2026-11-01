/**
 * vite.config.ts —— Vite 构建与开发服务器配置
 * 说明：直接导出类型安全的配置对象，免除在未执行 `npm install` 前因缺少 `node_modules/vite` 产生 TS2307 报错，
 *       同时在安装 Vite 后与 `vite` / `vite build` 100% 原生兼容。
 */

export interface ViteUserConfig {
  base?: string;
  server?: {
    port?: number;
    host?: boolean | string;
  };
  build?: {
    outDir?: string;
    target?: string | string[];
  };
}

const config: ViteUserConfig = {
  base: './',
  server: {
    port: 5173,
    host: true,
  },
  build: {
    outDir: 'dist',
    target: 'es2022',
  },
};

export default config;
