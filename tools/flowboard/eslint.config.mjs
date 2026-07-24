/**
 * ESLint flat config — 扫 tools/flowboard/js/vis/ 的 no-undef + no-unused-vars
 *
 * 用法:
 *   npx eslint -c tools/flowboard/eslint.config.mjs tools/flowboard/js/vis/
 */

export default [
  // 全局忽略
  {
    ignores: ["**/three.min.js", "**/node_modules/**"],
  },

  // vis/ 目录专属规则
  {
    files: ["tools/flowboard/js/vis/**/*.js"],
    languageOptions: {
      ecmaVersion: 2022,
      sourceType: "module",
      globals: {
        // 浏览器全局
        THREE: "readonly",
        performance: "readonly",
        ResizeObserver: "readonly",
        console: "readonly",
        window: "readonly",
        document: "readonly",
        setTimeout: "readonly",
        clearTimeout: "readonly",
        setInterval: "readonly",
        clearInterval: "readonly",
        requestAnimationFrame: "readonly",
        cancelAnimationFrame: "readonly",
        URLSearchParams: "readonly",
        WebSocket: "readonly",
        EventSource: "readonly",
        fetch: "readonly",
        navigator: "readonly",
        location: "readonly",
        history: "readonly",
        Image: "readonly",
        ImageData: "readonly",
        CustomEvent: "readonly",
        MutationObserver: "readonly",
        IntersectionObserver: "readonly",
        // ES 内置
        Promise: "readonly",
        Map: "readonly",
        Set: "readonly",
        WeakMap: "readonly",
        WeakSet: "readonly",
        Symbol: "readonly",
        Proxy: "readonly",
        Reflect: "readonly",
        JSON: "readonly",
        Math: "readonly",
        Date: "readonly",
        RegExp: "readonly",
        Array: "readonly",
        Object: "readonly",
        String: "readonly",
        Number: "readonly",
        Boolean: "readonly",
        Function: "readonly",
        Error: "readonly",
        TypeError: "readonly",
        ReferenceError: "readonly",
        SyntaxError: "readonly",
        RangeError: "readonly",
        parseInt: "readonly",
        parseFloat: "readonly",
        isNaN: "readonly",
        isFinite: "readonly",
        undefined: "readonly",
        NaN: "readonly",
        Infinity: "readonly",
        BigInt: "readonly",
        globalThis: "readonly",
        // Node.js 全局（仅在测试中）
        process: "readonly",
        module: "readonly",
        exports: "readonly",
        require: "readonly",
        __dirname: "readonly",
        __filename: "readonly",
      },
    },
    rules: {
      "no-undef": "error",
      "no-unused-vars": ["warn", {
        "args": "none",
        "caughtErrors": "none",
        "varsIgnorePattern": "^_",
        "argsIgnorePattern": "^_",
      }],
      "no-duplicate-imports": "error",
      "no-const-assign": "error",
      "no-class-assign": "error",
    },
  },
];