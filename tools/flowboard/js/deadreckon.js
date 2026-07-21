// ═════════════════════════════════════════════════════════════════════
// deadreckon.js — 已迁移到 vis/core/DeadReckon.js
//
// Step 2 重构：算法实现已移到 vis/core/DeadReckon.js（SceneDirector 路径
// 下的新家）。本文件保留为 re-export shim，避免破坏 app.js / scene2d.js
// 等仍从 './deadreckon.js' import 的入口。
//
// Phase 4 清理时会删除本文件，届时 app.js / scene2d.js 改成直接从
// vis/core/DeadReckon.js import。
// ═════════════════════════════════════════════════════════════════════

export {
  LAMBDA_POS, LAMBDA_HEADING, _dr,
  initDeadReckon, updateDeadReckon, tickDeadReckon, getDeadReckonState,
} from './vis/core/DeadReckon.js';
