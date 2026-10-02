package core

import (
	"strconv"
	"strings"
	"time"

	"github.com/librescoot/librefsm"

	"vehicle-service/internal/fsm"
)

// Handlebar hold-to-lock ("auto-lock").
//
// The rider turns the handlebar all the way left and holds it there. When the
// scooter is genuinely at rest, a short countdown is published for the
// dashboard overlay and, once it elapses, the ordinary lock path runs
// (fsm.EvLock -> shutting-down -> standby, which also engages the steering
// lock). Releasing the handlebar, or any of the interactions that already
// reset the idle auto-standby timer, aborts the countdown.
//
// The countdown is edge-triggered on the off-place -> on-place transition and
// deliberately not re-armed from EnterParked. A rider who unlocks with the
// handlebar still turned left would otherwise re-lock a few seconds later
// without touching anything.
const (
	// defaultHandlebarAutoLockSeconds is the hold time when the setting is
	// absent. Zero disables the feature.
	defaultHandlebarAutoLockSeconds = 3
	// maxHandlebarAutoLockSeconds caps the setting. A longer hold buys no
	// safety and is only tiring to hold.
	maxHandlebarAutoLockSeconds = 5
	// handlebarAutoLockSpeedThresholdKmh is the largest engine-ECU speed still
	// considered stopped. The ECU reports km/h with quantization noise around
	// zero, so an exact zero comparison would never be satisfied.
	handlebarAutoLockSpeedThresholdKmh = 0.5
)

// handlebarAutoLockSettingKey is the settings-hash field for the hold time.
const handlebarAutoLockSettingKey = "scooter.handlebar-auto-lock-seconds"

// clampHandlebarAutoLock normalises a hold-time setting: negatives become 0
// (disabled) and values above the cap are clamped down.
func (v *VehicleSystem) clampHandlebarAutoLock(seconds int) int {
	if seconds < 0 {
		return 0
	}
	if seconds > maxHandlebarAutoLockSeconds {
		v.logger.Warnf("%s %d above maximum, clamping to %d", handlebarAutoLockSettingKey, seconds, maxHandlebarAutoLockSeconds)
		return maxHandlebarAutoLockSeconds
	}
	return seconds
}

// armHandlebarAutoLock starts the hold countdown. It is called on the
// off-place -> on-place edge only; every precondition lives here so the caller
// does not have to duplicate them.
func (v *VehicleSystem) armHandlebarAutoLock() {
	if v.machine == nil {
		return
	}

	v.mu.RLock()
	seconds := v.handlebarAutoLockSeconds
	override := v.handlebarUnlockedOverride
	unlocked := v.handlebarUnlocked
	v.mu.RUnlock()

	if seconds <= 0 || override || !unlocked {
		return
	}
	if v.getCurrentStateID() != fsm.StateParked {
		return
	}
	// Read the seatbox live rather than from the cache: this is the same
	// source IsSeatboxClosed uses for the FSM guard, so the countdown can
	// never arm for a lock the FSM would divert into waiting-seatbox. An
	// unreadable sensor fails closed.
	seatboxClosed, err := v.io.ReadDigitalInput("seatbox_lock_sensor")
	if err != nil {
		v.logger.Warnf("Handlebar auto-lock: failed to read seatbox sensor: %v", err)
		return
	}
	if !seatboxClosed {
		return
	}
	if !v.handlebarAutoLockSpeedOK() {
		return
	}

	duration := time.Duration(seconds) * time.Second
	deadline := time.Now().Add(duration)

	v.mu.Lock()
	if v.handlebarAutoLockTimer != nil {
		v.handlebarAutoLockTimer.Stop()
	}
	v.handlebarAutoLockDeadline = deadline
	v.handlebarAutoLockTimer = time.AfterFunc(duration, v.handlebarAutoLockExpired)
	v.mu.Unlock()

	v.logger.Infof("Handlebar held in the lock position: locking in %d seconds unless released", seconds)
	if err := v.redis.PublishAutoLockDeadline(deadline); err != nil {
		v.logger.Warnf("Failed to publish auto-lock deadline: %v", err)
	}
}

// cancelHandlebarAutoLock disarms the hold countdown and clears the published
// deadline so the dashboard overlay disappears immediately. Safe to call when
// nothing is armed.
func (v *VehicleSystem) cancelHandlebarAutoLock() {
	v.mu.Lock()
	timer := v.handlebarAutoLockTimer
	v.handlebarAutoLockTimer = nil
	armed := !v.handlebarAutoLockDeadline.IsZero()
	v.handlebarAutoLockDeadline = time.Time{}
	v.mu.Unlock()

	if timer == nil && !armed {
		return
	}
	if timer != nil {
		timer.Stop()
	}
	if err := v.redis.ClearAutoLockDeadline(); err != nil {
		v.logger.Warnf("Failed to clear auto-lock deadline: %v", err)
	}
}

// handlebarAutoLockExpired runs when the hold countdown elapses. It re-checks
// the cheap preconditions before dispatching the same event as an explicit
// lock request.
func (v *VehicleSystem) handlebarAutoLockExpired() {
	v.mu.Lock()
	v.handlebarAutoLockTimer = nil
	override := v.handlebarUnlockedOverride
	v.mu.Unlock()

	if override || v.getCurrentStateID() != fsm.StateParked {
		v.cancelHandlebarAutoLock()
		return
	}

	v.logger.Infof("Handlebar auto-lock: hold complete, locking")
	v.mu.Lock()
	v.handlebarAutoLockDeadline = time.Time{}
	v.mu.Unlock()
	if err := v.redis.ClearAutoLockDeadline(); err != nil {
		v.logger.Warnf("Failed to clear auto-lock deadline: %v", err)
	}
	v.machine.Send(librefsm.Event{ID: fsm.EvLock})
}

// handlebarAutoLockSpeedOK reports whether the engine-ECU speed reading permits
// an auto-lock. A missing or unparseable field counts as stopped: parked
// already implies a deployed kickstand and an engaged engine brake, so the
// speed check only has to veto a positive reading, not prove zero.
func (v *VehicleSystem) handlebarAutoLockSpeedOK() bool {
	raw, err := v.redis.GetHashField("engine-ecu", "speed")
	if err != nil || raw == "" {
		v.logger.Debugf("Handlebar auto-lock: no engine-ecu speed reading (%v), relying on parked state", err)
		return true
	}
	speed, parseErr := strconv.ParseFloat(strings.TrimSpace(raw), 64)
	if parseErr != nil {
		v.logger.Warnf("Handlebar auto-lock: unparseable engine-ecu speed %q, relying on parked state", raw)
		return true
	}
	if speed < 0 {
		speed = -speed
	}
	if speed > handlebarAutoLockSpeedThresholdKmh {
		v.logger.Infof("Handlebar auto-lock suppressed: speed %.1f km/h", speed)
		return false
	}
	return true
}
