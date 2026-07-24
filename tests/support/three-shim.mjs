/**
 * three-shim.mjs — 最小假 THREE，供 Node.js 测试加载 vis/ 模块
 *
 * 用递归 Proxy 实现"任何属性访问/调用/构造都返回自身"的自动 mock，
 * 无需枚举所有 THREE API。vis/ 模块的 import * as THREE from 'three'
 * 和 window.THREE 都会被解析为此 shim。
 *
 * 设计要点：
 *  - valueOf / Symbol.toPrimitive 返回 0，使算术运算（rotation.x += 0.016）不抛错
 *  - 常见类型守卫属性（.isMesh / .isMaterial 等）返回 undefined（falsy）
 *  - .children 返回 []（空数组），.name 返回 ''（空串）
 *  - instanceof 检查（Symbol.hasInstance）返回 true，避免分支走错
 *  - Array.isArray() 检查返回 false（Proxy 不是数组）
 *
 * 用法：
 *   node --import ./tests/support/three-preload.mjs tests/vis_module_load.test.mjs
 */

// 递归 Proxy 处理器
const handler = {
  get(target, prop) {
    // ── 类型守卫属性：返回 undefined（falsy），避免触发 isMesh 等分支 ──
    if (prop === 'isMesh' || prop === 'isMaterial' ||
        prop === 'isMeshPhysicalMaterial' || prop === 'isMeshStandardMaterial' ||
        prop === 'isMeshBasicMaterial' || prop === 'isMeshPhongMaterial' ||
        prop === 'isMeshToonMaterial' || prop === 'isSpriteMaterial' ||
        prop === 'isGroup' || prop === 'isObject3D' ||
        prop === 'isBufferGeometry' || prop === 'isInstancedMesh' ||
        prop === 'isSkinnedMesh' || prop === 'isBone' ||
        prop === 'isLine' || prop === 'isPoints' || prop === 'isSprite' ||
        prop === 'isCamera' || prop === 'isLight' || prop === 'isScene') {
      return undefined;
    }
    // ── 常见字符串属性：返回空串 ──
    if (prop === 'name' || prop === 'type' || prop === 'uuid') {
      return '';
    }
    // ── 常见数组属性：返回空数组 ──
    if (prop === 'children') {
      return [];
    }
    // ── 常见布尔属性 ──
    if (prop === 'visible' || prop === 'castShadow' || prop === 'receiveShadow' ||
        prop === 'frustumCulled' || prop === 'matrixAutoUpdate' ||
        prop === 'matrixWorldNeedsUpdate') {
      return true;
    }
    // ── 数值属性：返回 0 ──
    if (prop === 'length' || prop === 'id') {
      return 0;
    }
    // ── 特殊符号 ──
    if (prop === Symbol.toPrimitive) {
      return () => 0;
    }
    if (prop === 'valueOf') {
      return () => 0;
    }
    if (prop === 'toString') {
      return () => '[THREE proxy]';
    }
    if (prop === Symbol.hasInstance) {
      return () => true;
    }
    if (prop === Symbol.iterator) {
      return function* () { /* empty */ };
    }
    // ── 默认：返回新 Proxy ──
    return createProxy();
  },

  apply(target, thisArg, args) {
    return createProxy();
  },

  construct(target, args) {
    return createProxy();
  },

  set(target, prop, value) {
    return true; // 静默接受任何赋值
  },
};

function createProxy() {
  return new Proxy(function () { return createProxy(); }, handler);
}

// 根 THREE 对象（也是 Proxy）
const THREE = createProxy();

// 挂载全局 export
export default THREE;
export { THREE };

// 常用导出（具名导入兼容）
export const Scene = THREE;
export const Group = THREE;
export const Mesh = THREE;
export const Object3D = THREE;
export const BoxGeometry = THREE;
export const PlaneGeometry = THREE;
export const CylinderGeometry = THREE;
export const SphereGeometry = THREE;
export const CircleGeometry = THREE;
export const ConeGeometry = THREE;
export const TorusGeometry = THREE;
export const BufferGeometry = THREE;
export const InstancedMesh = THREE;
export const SkinnedMesh = THREE;
export const Bone = THREE;
export const Line = THREE;
export const LineBasicMaterial = THREE;
export const LineSegments = THREE;
export const Points = THREE;
export const Sprite = THREE;
export const SpriteMaterial = THREE;

export const MeshStandardMaterial = THREE;
export const MeshPhysicalMaterial = THREE;
export const MeshBasicMaterial = THREE;
export const MeshPhongMaterial = THREE;
export const MeshToonMaterial = THREE;
export const MeshLambertMaterial = THREE;
export const ShaderMaterial = THREE;
export const RawShaderMaterial = THREE;

export const Vector2 = THREE;
export const Vector3 = THREE;
export const Vector4 = THREE;
export const Euler = THREE;
export const Quaternion = THREE;
export const Matrix4 = THREE;
export const Matrix3 = THREE;
export const Color = THREE;
export const Box3 = THREE;
export const Sphere = THREE;
export const Raycaster = THREE;
export const Plane = THREE;
export const Frustum = THREE;

export const CatmullRomCurve3 = THREE;
export const CubicBezierCurve3 = THREE;
export const QuadraticBezierCurve3 = THREE;
export const LineCurve3 = THREE;
export const SplineCurve = THREE;

export const BufferAttribute = THREE;
export const Float32BufferAttribute = THREE;
export const Uint16BufferAttribute = THREE;
export const Uint32BufferAttribute = THREE;

export const Camera = THREE;
export const PerspectiveCamera = THREE;
export const OrthographicCamera = THREE;
export const ArrayCamera = THREE;

export const AmbientLight = THREE;
export const DirectionalLight = THREE;
export const PointLight = THREE;
export const SpotLight = THREE;
export const HemisphereLight = THREE;
export const RectAreaLight = THREE;
export const Light = THREE;
export const LightShadow = THREE;
export const DirectionalLightShadow = THREE;
export const SpotLightShadow = THREE;
export const PointLightShadow = THREE;
export const CameraHelper = THREE;

export const WebGLRenderer = THREE;
export const WebGL1Renderer = THREE;
export const WebGLRenderTarget = THREE;
export const WebGLCubeRenderTarget = THREE;

export const PMREMGenerator = THREE;
export const TextureLoader = THREE;
export const CubeTextureLoader = THREE;
export const ImageBitmapLoader = THREE;
export const FileLoader = THREE;
export const LoadingManager = THREE;
export const GLTFLoader = THREE;
export const RGBELoader = THREE;
export const DRACOLoader = THREE;
export const KTX2Loader = THREE;
export const EXRLoader = THREE;
export const OBJLoader = THREE;
export const MTLLoader = THREE;
export const FBXLoader = THREE;

export const Texture = THREE;
export const DataTexture = THREE;
export const CanvasTexture = THREE;
export const CompressedTexture = THREE;
export const CubeTexture = THREE;
export const VideoTexture = THREE;
export const DepthTexture = THREE;

export const Clock = THREE;
export const MathUtils = THREE;
export const AnimationClip = THREE;
export const AnimationMixer = THREE;
export const AnimationAction = THREE;
export const KeyframeTrack = THREE;
export const PropertyBinding = THREE;
export const AnimationUtils = THREE;

export const AxesHelper = THREE;
export const GridHelper = THREE;
export const Box3Helper = THREE;
export const BoxHelper = THREE;
export const SkeletonHelper = THREE;
export const ArrowHelper = THREE;
export const PolarGridHelper = THREE;
export const VertexNormalsHelper = THREE;
export const FaceNormalsHelper = THREE;

// 常量导出
export const DoubleSide = 2;
export const FrontSide = 0;
export const BackSide = 1;
export const SRGBColorSpace = 'srgb';
export const LinearSRGBColorSpace = 'srgb-linear';
export const NoColorSpace = '';
export const DisplayP3ColorSpace = 'display-p3';

export const AdditiveBlending = 2;
export const NormalBlending = 1;
export const SubtractiveBlending = 3;
export const MultiplyBlending = 4;
export const NoBlending = 0;
export const CustomBlending = 5;

export const UnsignedByteType = 1009;
export const FloatType = 1015;
export const HalfFloatType = 1016;
export const UnsignedShortType = 1011;
export const UnsignedIntType = 1013;

export const LinearFilter = 1006;
export const NearestFilter = 1003;
export const LinearMipmapLinearFilter = 1008;
export const NearestMipmapLinearFilter = 1007;
export const LinearMipmapNearestFilter = 1009;
export const NearestMipmapNearestFilter = 1004;

export const ClampToEdgeWrapping = 1001;
export const RepeatWrapping = 1000;
export const MirroredRepeatWrapping = 1002;

export const EquirectangularReflectionMapping = 1;
export const CubeReflectionMapping = 2;
export const CubeRefractionMapping = 3;
export const UVMapping = 300;

export const PCFSoftShadowMap = 2;
export const PCFShadowMap = 1;
export const BasicShadowMap = 0;

export const LoopOnce = 2200;
export const LoopRepeat = 2201;
export const LoopPingPong = 2202;

export const AlwaysStencilFunc = 519;
export const EqualStencilFunc = 514;
export const NotEqualStencilFunc = 517;
export const ReplaceStencilOp = 7687;

export const StaticDrawUsage = 35044;
export const DynamicDrawUsage = 35048;
export const StreamDrawUsage = 35040;

export const RGBAFormat = 1023;
export const RGBFormat = 1022;
export const DepthFormat = 1026;
export const RGBA16F = 1028;

export const NeverDepth = 512;
export const AlwaysDepth = 513;
export const LessDepth = 515;
export const LessEqualDepth = 516;
export const EqualDepth = 514;