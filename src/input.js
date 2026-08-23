import { clamp } from './core/math.js';
import { playerSteerFromScreenAxis } from './core/conventions.js';
import { KeyboardDynamics } from './input/KeyboardDynamics.js';

const GAME_CODES = new Set([
  'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space',
  'KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyR', 'KeyC', 'KeyM', 'KeyT',
  'KeyE', 'KeyQ', 'KeyP', 'KeyG', 'KeyN', 'Tab',
  'Digit1', 'Digit2', 'Digit3', 'Digit4', 'Digit5',
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

    this._onKeyDown = (event) => {
      if (GAME_CODES.has(event.code)) {
        // Prevent scrolling for game controls, allow normal tab if needed or prevent
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

    if (typeof window !== 'undefined') {
      window.addEventListener('keydown', this._onKeyDown, { passive: false });
      window.addEventListener('keyup', this._onKeyUp);
      window.addEventListener('pointerdown', this._onPointer);
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
    return {
      forward: (this.held('KeyW') ? 1 : 0) - (this.held('KeyS') ? 1 : 0),
      right: (this.held('KeyD') ? 1 : 0) - (this.held('KeyA') ? 1 : 0),
      up: (this.held('KeyE') ? 1 : 0) - (this.held('KeyQ') ? 1 : 0),
      yaw: (this.held('ArrowRight') ? 1 : 0) - (this.held('ArrowLeft') ? 1 : 0),
      pitch: (this.held('ArrowUp') ? 1 : 0) - (this.held('ArrowDown') ? 1 : 0),
      boost: this.held('ShiftLeft', 'ShiftRight')
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
    this.keyboardDynamics.reset();
  }

  dispose() {
    if (typeof window === 'undefined') return;
    window.removeEventListener('keydown', this._onKeyDown);
    window.removeEventListener('keyup', this._onKeyUp);
    window.removeEventListener('pointerdown', this._onPointer);
  }
}
