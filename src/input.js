import { clamp } from './core/math.js';
import { playerSteerFromScreenAxis } from './core/conventions.js';
import { KeyboardDynamics } from './input/KeyboardDynamics.js';

const GAME_CODES = new Set([
  'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space',
  'KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyR', 'KeyC', 'KeyV', 'KeyM', 'KeyT',
  'KeyE', 'KeyQ', 'KeyP', 'KeyG', 'KeyN', 'KeyJ', 'KeyF', 'KeyU', 'Tab',
  'Digit1', 'Digit2', 'Digit3', 'Digit4', 'Digit5', 'Digit6', 'Digit7',
  'BracketLeft', 'BracketRight', 'Semicolon', 'Quote', 'Comma', 'Period',
  'F1', 'F2', 'F3', 'F4', 'F5', 'F6', 'F7'
]);

const padValue = (button) => clamp(typeof button === 'number' ? button : button?.value ?? 0, 0, 1);

export class InputManager {
  constructor(onGesture) {
    this.keys = new Set();
    this.pressed = new Set();
    this.onGesture = onGesture;
    this.keyboardDynamics = new KeyboardDynamics();

    this.mouseDeltaX = 0;
    this.mouseDeltaY = 0;
    this.flightSpeed = 32;

    this._onKeyDown = (event) => {
      if (GAME_CODES.has(event.code)) {
        if (event.code !== 'Tab' || !event.shiftKey) event.preventDefault();
      }
      if (!this.keys.has(event.code)) this.pressed.add(event.code);
      this.keys.add(event.code);
      this.onGesture?.();
    };

    this._onKeyUp = (event) => {
      this.keys.delete(event.code);
    };

    this._onPointer = () => {
      this.onGesture?.();
    };

    this._onMouseMove = (event) => {
      if (document.pointerLockElement) {
        this.mouseDeltaX += event.movementX || 0;
        this.mouseDeltaY += event.movementY || 0;
      }
    };

    this._onWheel = (event) => {
      if (document.pointerLockElement) {
        event.preventDefault();
        const delta = Math.sign(event.deltaY);
        this.flightSpeed = clamp(this.flightSpeed - delta * 6, 8, 160);
      }
    };

    if (typeof window !== 'undefined') {
      window.addEventListener('keydown', this._onKeyDown, { passive: false });
      window.addEventListener('keyup', this._onKeyUp);
      window.addEventListener('pointerdown', this._onPointer);
      window.addEventListener('mousemove', this._onMouseMove);
      window.addEventListener('wheel', this._onWheel, { passive: false });
    }
  }

  held(...codes) {
    return codes.some((code) => this.keys.has(code));
  }

  consume(code) {
    if (!this.pressed.has(code)) return false;
    this.pressed.delete(code);
    return true;
  }

  isInteracting() {
    return this.held('KeyW', 'KeyA', 'KeyS', 'KeyD', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space');
  }

  isPointerLocked() {
    return typeof document !== 'undefined' && Boolean(document.pointerLockElement);
  }

  requestPointerLock(element) {
    if (!element || typeof document === 'undefined') return;
    try {
      if (document.pointerLockElement !== element) {
        element.requestPointerLock?.();
      }
    } catch {
      // Ignored if user gesture required
    }
  }

  exitPointerLock() {
    if (typeof document === 'undefined') return;
    try {
      if (document.pointerLockElement) {
        document.exitPointerLock?.();
      }
    } catch {
      // Ignored
    }
  }

  keyboardRaw() {
    const screenAxis = (this.held('KeyD', 'ArrowRight') ? 1 : 0) - (this.held('KeyA', 'ArrowLeft') ? 1 : 0);
    return {
      throttle: this.held('KeyW', 'ArrowUp') ? 1 : 0,
      brake: this.held('KeyS', 'ArrowDown') ? 1 : 0,
      steer: playerSteerFromScreenAxis(screenAxis),
      handbrake: this.held('Space') ? 1 : 0
    };
  }

  freeCameraRaw() {
    const dx = this.mouseDeltaX;
    const dy = this.mouseDeltaY;
    this.mouseDeltaX = 0;
    this.mouseDeltaY = 0;

    return {
      forward: (this.held('KeyW') ? 1 : 0) - (this.held('KeyS') ? 1 : 0),
      right: (this.held('KeyD') ? 1 : 0) - (this.held('KeyA') ? 1 : 0),
      up: (this.held('KeyE', 'Space') ? 1 : 0) - (this.held('KeyQ', 'KeyC') ? 1 : 0),
      yaw: (this.held('ArrowRight') ? 1 : 0) - (this.held('ArrowLeft') ? 1 : 0),
      pitch: (this.held('ArrowUp') ? 1 : 0) - (this.held('ArrowDown') ? 1 : 0),
      boost: this.held('ShiftLeft', 'ShiftRight'),
      slow: this.held('ControlLeft', 'ControlRight'),
      baseSpeed: this.flightSpeed,
      mouseLook: { dx, dy }
    };
  }

  _gamepadRaw() {
    const pads = typeof navigator === 'undefined' ? [] : (navigator.getGamepads?.() ?? []);
    const gamepad = [...pads].find(Boolean);
    if (!gamepad) return null;
    const rawAxis = gamepad.axes?.[0] ?? 0;
    const screenAxis = Math.abs(rawAxis) > 0.12 ? rawAxis : 0;
    return {
      throttle: Math.max(padValue(gamepad.buttons?.[7]), padValue(gamepad.buttons?.[0])),
      brake: Math.max(padValue(gamepad.buttons?.[6]), padValue(gamepad.buttons?.[1])),
      steer: playerSteerFromScreenAxis(screenAxis),
      handbrake: padValue(gamepad.buttons?.[2])
    };
  }

  controls(vehicle = null, dt = 1 / 120) {
    const keyboard = this.keyboardDynamics.update(this.keyboardRaw(), vehicle, dt);
    const gamepad = this._gamepadRaw();
    if (!gamepad) return keyboard;

    return {
      throttle: Math.max(keyboard.throttle, gamepad.throttle),
      brake: Math.max(keyboard.brake, gamepad.brake),
      steer: Math.abs(gamepad.steer) > Math.abs(keyboard.steer) + 0.04 ? gamepad.steer : keyboard.steer,
      handbrake: Math.max(keyboard.handbrake, gamepad.handbrake)
    };
  }

  reset() {
    this.keys.clear();
    this.pressed.clear();
    this.mouseDeltaX = 0;
    this.mouseDeltaY = 0;
    this.keyboardDynamics.reset();
  }

  dispose() {
    if (typeof window === 'undefined') return;
    window.removeEventListener('keydown', this._onKeyDown);
    window.removeEventListener('keyup', this._onKeyUp);
    window.removeEventListener('pointerdown', this._onPointer);
    window.removeEventListener('mousemove', this._onMouseMove);
    window.removeEventListener('wheel', this._onWheel);
  }
}
