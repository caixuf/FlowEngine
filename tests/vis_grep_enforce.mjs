/**
 * vis_grep_enforce.mjs — 坐标约定 + 渲染门禁违规检测
 *
 * A-3 重构：扫描范围从 view/ 扩到整个 js/ 目录（含 core/director/math/store/
 * view/main.js + 顶层 app.js/scene2d.js/models.js 等），紧缩豁免规则：
 *   - 移除泛化词（angle / len / PI / SIZE / line / screen 等）
 *   - 引入 fileExempt（按文件路径精确豁免"实现本身"和"非 3D 上下文"文件）
 *   - 保留 // exempt 行标记作为通用逃生口
 *
 * 检测项：
 *   1. 裸 -y 翻转（如 `z: -(n[2])`）
 *   2. 裸 atan2 朝向计算
 *   3. 裸 Math.sin / Math.cos 手算偏移
 *   4. 裸 .position.set 配 sin/cos 魔法数
 *   5. 外网资产依赖（https://）
 *   6. 车身材质金属度过高（metalness >= 0.55）
 *
 * 命中即 FAIL。
 * 豁免方式：
 *   - 行内含 `// exempt` 显式标记
 *   - 行内容命中 exempt 词表（如 `Coord.`、`wheel` 等具名零件）
 *   - 文件路径命中 fileExempt（如 `math/Coord.js` 是实现本身）
 *
 * 跑法：node tests/vis_grep_enforce.mjs
 */

import { execSync } from 'child_process';
import { resolve, dirname } from 'path';
import { fileURLToPath } from 'url';

const __dirname = dirname(fileURLToPath(import.meta.url));
// A-3: 全部 JS 目录 —— 从 view/ 扩到整个 js/ 树
const JS_DIR = resolve(__dirname, '../tools/flowboard/js');

const RULES = [
  {
    name: '裸 -y 翻转 (z: -(n[2]) 等)',
    pattern: 'z:\\s*-\\(',
    desc: '应使用 Coord.worldToThree 替代手写 ENU→THREE 翻转',
    dir: JS_DIR,
    exempt: /\/\/\s*exempt|worldToThree|Coord\./,
    // Curve.sampleEdgeNodes 内部做 ENU→THREE 交换，是 Coord 之外的第二处合法翻转
    fileExempt: /\/math\/Curve\.js$/,
  },
  {
    name: '裸 atan2 朝向计算',
    pattern: 'Math\\.atan2',
    desc: '应使用 Coord.directionToRotationY 替代裸 atan2',
    dir: JS_DIR,
    exempt: /\/\/\s*exempt|directionToRotationY|Coord\./,
    // Coord.js 是 directionToRotationY 实现本身；
    // app.js / scene2d.js 是 GPS history 推算 + 2D canvas 箭头，非 3D 朝向
    fileExempt: /\/math\/Coord\.js$|\/app\.js$|\/scene2d\.js$/,
  },
  {
    name: '裸 Math.sin/cos 手算偏移',
    pattern: 'Math\\.(sin|cos)',
    desc: '应使用 Coord.forwardENU / offsetAlongNormal 替代手算正余弦',
    dir: JS_DIR,
    // A-3 紧缩：移除 angle / len / PI / Math.PI / Math.random / SIZE /
    //   tangentToNormal / Sobel / _asphalt / _buildAsphalt / noise / 裂缝 / 微裂纹
    //   这些词太泛，会误豁免真实违规。
    // 保留 forwardENU / offsetAlongNormal / Coord. 作为合法 API 调用标记。
    // ctx\. 用于 2D Canvas 上下文（与 3D 坐标变换无关）。
    exempt: /\/\/\s*exempt|forwardENU|offsetAlongNormal|Coord\.|ctx\./,
    // Coord.js = 实现本身；
    // DeadReckon.js = 热路径外推（避免函数调用开销）；
    // SkyEnv.js = 太阳位置（圆弧轨迹，非道路偏移）；
    // CameraRig.js = 相机相对 ego 位姿（非路上物体放置）；
    // app.js / scene2d.js / models.js = 2D canvas / GPS / 动画
    fileExempt: /\/math\/Coord\.js$|\/core\/DeadReckon\.js$|\/core\/SkyEnv\.js$|\/core\/CameraRig\.js$|\/app\.js$|\/scene2d\.js$|\/models\.js$/,
  },
  {
    name: '裸 .position.set 配魔法数',
    pattern: '\\.position\\.set\\(.*Math\\.(sin|cos)',
    desc: '应使用 Coord.placeOnRoad / offsetAlongNormal 替代 position.set 配手算',
    dir: JS_DIR,
    exempt: /\/\/\s*exempt|Coord\.|placeOnRoad/,
  },
  // ── 渲染门禁：材质 + 外网依赖 ──
  {
    name: '外网资产依赖 (https:// 或 unpkg)',
    pattern: 'https://',
    desc: '禁止从外网加载纹理/HDRI/模型，离线必须自洽。需豁免的行加 // exempt',
    dir: JS_DIR,
    exempt: /\/\/\s*exempt|github\.com|Source Han|fonts\.googleapis|trae-api/,
  },
  {
    name: '车身材质金属度过高 (metalness >= 0.55)',
    pattern: 'metalness:\\s*0\\.[5-9][5-9]|metalness:\\s*0\\.[6-9]\\d*|metalness:\\s*[1-9]',
    desc: '车漆 metalness 应 ≤ 0.5。轮毂/镀铬/玻璃等非车身材质豁免行加 // exempt',
    dir: JS_DIR,
    // A-3 紧缩：移除 `line`（太泛，匹配 LineSegments / linear / polyline 等）
    //   和 `screen`（变量 screenMat 已是 0.5 不触发，词义不清）。
    // `lineMat` 替代 `line` —— 道路标线材质（非车身），变量名精确。
    exempt: /\/\/\s*exempt|wheel|hub|chrome|glass|trim|bezel|splitter|spoiler|rubber|tread|tire|axle|pole|rail|metalRail|lineMat|fog|lamp|light|sensor|grille|radiator|bumper/,
  },
];

let totalFail = 0;

for (const rule of RULES) {
  console.log(`\n检查: ${rule.name}`);
  const scanDir = rule.dir || JS_DIR;
  try {
    const cmd = `grep -rnE "${rule.pattern}" "${scanDir}"`;
    const output = execSync(cmd, { encoding: 'utf-8', stdio: ['pipe', 'pipe', 'pipe'] });
    const lines = output.trim().split('\n').filter(Boolean);
    let violations = 0;

    for (const line of lines) {
      // 提取文件路径和行号
      const [filePath, ...rest] = line.split(':');
      const lineContent = rest.join(':');

      // A-3: 文件级精确豁免（如 Coord.js 是实现本身）
      if (rule.fileExempt && rule.fileExempt.test(filePath)) {
        continue;
      }

      // 行内容词表豁免
      if (rule.exempt && rule.exempt.test(lineContent)) {
        continue;
      }

      // 检查是否是注释行（// 在匹配之前）
      const codeBeforeMatch = lineContent.substring(0, lineContent.search(new RegExp(rule.pattern)));
      if (/\/\//.test(codeBeforeMatch)) {
        continue;
      }

      violations++;
      console.log(`  VIOLATION  ${filePath}: ${lineContent.trim().substring(0, 100)}`);
      console.log(`             → ${rule.desc}`);
    }

    if (violations === 0) {
      console.log('  PASS  无违规');
    } else {
      console.log(`  FAIL  ${violations} 处违规`);
      totalFail += violations;
    }
  } catch (e) {
    // grep 无匹配时返回非零，这是正常的
    if (e.status === 1) {
      console.log('  PASS  无违规');
    } else {
      console.log(`  ERROR  ${e.message}`);
      totalFail++;
    }
  }
}

console.log(`\n=== 坐标约定 + 渲染门禁检测完成: ${totalFail} 处违规 ===`);
process.exit(totalFail > 0 ? 1 : 0);
