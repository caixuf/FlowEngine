/**
 * models.js — glTF model cache for FlowBoard 3D scene
 *
 * Loads vehicle/pedestrian .gltf files via Three.js GLTFLoader.
 * Falls back to programmatic geometry when GLTFLoader is unavailable
 * or a model file fails to load.
 *
 * Usage:
 *   import { initModelCache, getModel } from './models.js';
 *   await initModelCache();
 *   const sedan = getModel('sedan').clone();
 *   scene.add(sedan);
 */

import { _buildSedan, _buildObstacle } from './utils.js';

const THREE = window.THREE;

/** Model registry: { name: THREE.Group or null (fallback) } */
const _cache = {};

/** Loading state */
let _ready = false;
let _loadError = null;

const MODEL_NAMES = ['sedan', 'truck', 'suv', 'pedestrian'];

/**
 * Preload all glTF models from /tools/flowboard/models/<name>.gltf.
 * If GLTFLoader is unavailable, all models stay null and getModel() returns fallback.
 * Returns a promise that resolves when all models are loaded or failed.
 */
export function initModelCache() {
  if (_ready) return Promise.resolve();
  if (window._gltfLoaderUnavailable) {
    _ready = true;
    return Promise.resolve();
  }
  if (!THREE || !THREE.GLTFLoader) {
    window._gltfLoaderUnavailable = true;
    _ready = true;
    return Promise.resolve();
  }

  return new Promise(function(resolve) {
    var loader = new THREE.GLTFLoader();
    var pending = MODEL_NAMES.length;
    var anyLoaded = false;

    function onModelLoaded(name, gltf) {
      // Extract the scene group; merge all nodes into a single group
      var group = new THREE.Group();
      gltf.scene.traverse(function(child) {
        if (child.isMesh) {
          // Clone the mesh material so each instance can have its own color
          var clone = child.clone();
          clone.material = child.material.clone();
          group.add(clone);
        }
      });
      // Set userData for type identification
      group.userData.isVehicle = true;
      group.userData.modelType = name;
      _cache[name] = group;
      anyLoaded = true;
      pending--;
      if (pending <= 0) { _ready = true; resolve(); }
    }

    function onModelError(name, err) {
      console.warn('[models] ' + name + ' load failed: ' + (err.message || err) + ' — using programmatic fallback');
      _cache[name] = null;
      pending--;
      if (pending <= 0) { _ready = true; resolve(); }
    }

    for (var i = 0; i < MODEL_NAMES.length; i++) {
      var name = MODEL_NAMES[i];
      _cache[name] = null;
      loader.load(
        '/tools/flowboard/models/' + name + '.gltf',
        function(n) { return function(g) { onModelLoaded(n, g); }; }(name),
        undefined,
        function(n) { return function(e) { onModelError(n, e); }; }(name)
      );
    }
  });
}

/**
 * Get a cached model group by type name.
 * Returns a THREE.Group (clone before adding to scene).
 * If glTF model is unavailable, returns null (caller uses programmatic fallback).
 *
 * @param {string} type  'sedan', 'truck', 'suv', 'pedestrian', or undefined
 * @returns {THREE.Group|null}
 */
export function getModel(type) {
  var name = type || 'car';
  // Map type names to model names
  switch (name) {
    case 'car':    name = 'sedan'; break;
    case 'truck':  name = 'truck'; break;
    case 'suv':    name = 'suv'; break;
    case 'pedestrian': name = 'pedestrian'; break;
    default:       name = 'sedan';
  }
  var model = _cache[name];
  if (model) {
    var clone = model.clone();
    // Scale: glTF models are built in meters (1:1 with scene)
    // Reset any pre-applied transforms
    clone.scale.set(1, 1, 1);
    return clone;
  }
  return null;
}

/**
 * Build a vehicle group for use as ego car.
 * Tries glTF first, falls back to programmatic sedan.
 */
export function buildEgoCar(color) {
  var model = getModel('sedan');
  if (model) {
    // Recolor all meshes
    model.traverse(function(c) {
      if (c.isMesh && c.material && c.material.color) {
        c.material.color.setHex(color || 0x4488dd);
      }
    });
    return model;
  }
  return _buildSedan(color || 0x4488dd, 0x3377bb);
}

/**
 * Build an obstacle group for NPC vehicles/pedestrians.
 * Tries glTF model first, falls back to programmatic box geometry.
 */
export function buildObstacleGroup(type, color) {
  var model = getModel(type);
  if (model) {
    // Apply obstacle color
    var c = color || 0xff9944;
    model.traverse(function(ch) {
      if (ch.isMesh && ch.material && ch.material.color) {
        ch.material.color.setHex(c);
      }
    });
    return model;
  }
  return _buildObstacle(type || 'car', color || 0xff9944);
}
