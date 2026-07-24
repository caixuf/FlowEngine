/**
 * vis_grep_enforce.mjs — 坐标约定违规检测
 *
 * 用 grep 扫描 js/vis/view/ 目录，检测裸坐标变换违规：
 *   1. 裸 -y 翻转（如 `z: -(n[2])`）
 *   2. 裸 atan2 朝向计算
 *   3. 裸 position.set 配魔法数
 *   4. 裸 Math.sin / Math.cos 手算偏移
 *
 * 命中即 FAIL（注释豁免：行内含 // exempt 或 // Coord 的豁免）。
 * 跑法：node tests/vis_grep_enforce.mjs
 */

import { execSync } from 'child_process';
import { readFileSync } from 'fs';
import { resolve, dirname } from 'path';
import { fileURLToPath } from 'url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const VIEW_DIR = resolve(__dirname, '../tools/flowboard/js/vis/view');
const JS_DIR = resolve(__dirname, '../tools/flowboard/js');

const RULES = [
  {
    name: '裸 -y 翻转 (z: -(n[2]) 等)',
    pattern: 'z:\\s*-\\(',
    desc: '应使用 Coord.worldToThree 替代手写 ENU→THREE 翻转',
    dir: VIEW_DIR,
    exempt: /worldToThree|Coord\./,
  },
  {
    name: '裸 atan2 朝向计算',
    pattern: 'Math\\.atan2',
    desc: '应使用 Coord.directionToRotationY 替代裸 atan2',
    dir: VIEW_DIR,
    exempt: /directionToRotationY|Coord\./,
  },
  {
    name: '裸 Math.sin/cos 手算偏移',
    pattern: 'Math\\.(sin|cos)',
    desc: '应使用 Coord.forwardENU / offsetAlongNormal 替代手算正余弦',
    dir: VIEW_DIR,
    exempt: /forwardENU|offsetAlongNormal|Coord\.|tangentToNormal|Sobel|_asphalt|_buildAsphalt|noise|裂缝|微裂纹|SIZE|PI|Math\\.PI|Math\\.random|angle|len/,
  },
  {
    name: '裸 .position.set 配魔法数',
    pattern: '\\.position\\.set\\(.*Math\\.(sin|cos)',
    desc: '应使用 Coord.placeOnRoad / offsetAlongNormal 替代 position.set 配手算',
    dir: VIEW_DIR,
    exempt: /Coord\.|placeOnRoad/,
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
    exempt: /\/\/\s*exempt|wheel|hub|chrome|glass|trim|bezel|splitter|spoiler|line|rubber|tread|tire|axle|pole|rail|metalRail|screen|fog|lamp|light|sensor|grille|radiator|bumper/,
  },
];

let totalFail = 0;

for (const rule of RULES) {
  console.log(`\n检查: ${rule.name}`);
  const scanDir = rule.dir || VIEW_DIR;
  try {
    const cmd = `grep -rnE "${rule.pattern}" "${scanDir}"`;
    const output = execSync(cmd, { encoding: 'utf-8', stdio: ['pipe', 'pipe', 'pipe'] });
    const lines = output.trim().split('\n').filter(Boolean);
    let violations = 0;

    for (const line of lines) {
      // 提取文件路径和行号
      const [filePath, ...rest] = line.split(':');
      const lineContent = rest.join(':');

      // 豁免检查
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

console.log(`\n=== 坐标约定检测完成: ${totalFail} 处违规 ===`);
process.exit(totalFail > 0 ? 1 : 0);