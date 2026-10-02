package core

import (
	"testing"
	"time"

	"github.com/librescoot/librefsm"

	"vehicle-service/internal/fsm"
)

// autoLockSystem builds a parked-family system with the handlebar unlocked and
// the seatbox closed, i.e. the state the hold-to-lock gesture requires.
func autoLockSystem(t *testing.T, state librefsm.StateID) (*VehicleSystem, *mockHardwareIO, *mockMessagingClient) {
	t.Helper()
	v, io, redis := newTestVehicleSystem()
	io.setDigitalInput("kickstand", true)
	io.setDigitalInput("handlebar_lock_sensor", true) // unlocked
	io.setDigitalInput("seatbox_lock_sensor", true)   // closed
	io.setDigitalInput("handlebar_position", false)   // off the lock detent
	initTestFSM(t, v)
	t.Cleanup(func() {
		v.cancelHandlebarAutoLock()
		v.machine.Stop()
		v.cancelHandlebarLock()
		v.cancelHandlebarUnlock()
	})
	if err := v.machine.SetState(state); err != nil {
		t.Fatal(err)
	}
	return v, io, redis
}

func autoLockArmed(v *VehicleSystem) bool {
	v.mu.RLock()
	defer v.mu.RUnlock()
	return v.handlebarAutoLockTimer != nil
}

func autoLockDeadlineCount(redis *mockMessagingClient) int {
	redis.mu.Lock()
	defer redis.mu.Unlock()
	return len(redis.autoLockDeadlines)
}

func autoLockClearCount(redis *mockMessagingClient) int {
	redis.mu.Lock()
	defer redis.mu.Unlock()
	return redis.autoLockDeadlineClears
}

func waitForState(t *testing.T, v *VehicleSystem, want librefsm.StateID, timeout time.Duration) {
	t.Helper()
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if v.machine.CurrentState() == want {
			return
		}
		time.Sleep(time.Millisecond)
	}
	t.Fatalf("state did not reach %s (still %s)", want, v.machine.CurrentState())
}

func TestHandlebarAutoLock_ArmsWhenParkedAndClosed(t *testing.T) {
	v, _, redis := autoLockSystem(t, fsm.StateParked)

	if err := v.handleHandlebarPosition("handlebar_position", true); err != nil {
		t.Fatal(err)
	}
	if !autoLockArmed(v) {
		t.Fatal("expected the hold countdown to be armed")
	}
	if n := autoLockDeadlineCount(redis); n != 1 {
		t.Fatalf("expected one published auto-lock deadline, got %d", n)
	}
}

func TestHandlebarAutoLock_DoesNotArm(t *testing.T) {
	tests := []struct {
		name   string
		state  librefsm.StateID
		mutate func(t *testing.T, v *VehicleSystem, io *mockHardwareIO, redis *mockMessagingClient)
	}{
		{
			name:  "seatbox open",
			state: fsm.StateParked,
			mutate: func(t *testing.T, v *VehicleSystem, io *mockHardwareIO, redis *mockMessagingClient) {
				io.setDigitalInput("seatbox_lock_sensor", false)
			},
		},
		{
			name:  "feature disabled",
			state: fsm.StateParked,
			mutate: func(t *testing.T, v *VehicleSystem, io *mockHardwareIO, redis *mockMessagingClient) {
				v.mu.Lock()
				v.handlebarAutoLockSeconds = 0
				v.mu.Unlock()
			},
		},
		{
			name:  "service-mode override",
			state: fsm.StateParked,
			mutate: func(t *testing.T, v *VehicleSystem, io *mockHardwareIO, redis *mockMessagingClient) {
				v.mu.Lock()
				v.handlebarUnlockedOverride = true
				v.mu.Unlock()
			},
		},
		{
			name:  "scooter moving",
			state: fsm.StateParked,
			mutate: func(t *testing.T, v *VehicleSystem, io *mockHardwareIO, redis *mockMessagingClient) {
				redis.hashFields = map[string]string{"engine-ecu/speed": "12.5"}
			},
		},
		{
			name:  "not parked",
			state: fsm.StateReadyToDrive,
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			v, io, redis := autoLockSystem(t, tc.state)
			if tc.mutate != nil {
				tc.mutate(t, v, io, redis)
			}
			if err := v.handleHandlebarPosition("handlebar_position", true); err != nil {
				t.Fatal(err)
			}
			if autoLockArmed(v) {
				t.Fatal("countdown must not arm")
			}
			if n := autoLockDeadlineCount(redis); n != 0 {
				t.Fatalf("no deadline must be published, got %d", n)
			}
		})
	}
}

// A handlebar that is still locked takes the anti-trap unlock path instead of
// arming the countdown.
func TestHandlebarAutoLock_DoesNotArmWhenAlreadyLocked(t *testing.T) {
	v, io, redis := autoLockSystem(t, fsm.StateParked)
	io.setDigitalInput("handlebar_lock_sensor", false) // locked

	if err := v.handleHandlebarPosition("handlebar_position", true); err != nil {
		t.Fatal(err)
	}
	if autoLockArmed(v) {
		t.Fatal("countdown must not arm while the handlebar is locked")
	}
	if n := autoLockDeadlineCount(redis); n != 0 {
		t.Fatalf("no deadline must be published, got %d", n)
	}
}

func TestHandlebarAutoLock_ZeroSpeedStillArms(t *testing.T) {
	v, _, redis := autoLockSystem(t, fsm.StateParked)
	redis.hashFields = map[string]string{"engine-ecu/speed": "0.0"}

	if err := v.handleHandlebarPosition("handlebar_position", true); err != nil {
		t.Fatal(err)
	}
	if !autoLockArmed(v) {
		t.Fatal("a stopped engine-ECU reading must still arm")
	}
}

func TestHandlebarAutoLock_CancelOnHandlebarRelease(t *testing.T) {
	v, _, redis := autoLockSystem(t, fsm.StateParked)
	if err := v.handleHandlebarPosition("handlebar_position", true); err != nil {
		t.Fatal(err)
	}
	if !autoLockArmed(v) {
		t.Fatal("precondition: expected armed")
	}

	if err := v.handleHandlebarPosition("handlebar_position", false); err != nil {
		t.Fatal(err)
	}
	if autoLockArmed(v) {
		t.Fatal("releasing the handlebar must disarm the countdown")
	}
	if autoLockClearCount(redis) == 0 {
		t.Fatal("expected the published deadline to be cleared")
	}
}

func TestHandlebarAutoLock_CancelOnRiderInput(t *testing.T) {
	for _, channel := range []string{"brake_left", "kickstand", "seatbox_button"} {
		t.Run(channel, func(t *testing.T) {
			v, io, redis := autoLockSystem(t, fsm.StateParked)
			if err := v.handleHandlebarPosition("handlebar_position", true); err != nil {
				t.Fatal(err)
			}
			if !autoLockArmed(v) {
				t.Fatal("precondition: expected armed")
			}

			// The handler reads the lever state back from the IO layer.
			io.setDigitalInput(channel, true)
			if err := v.handleInputChange(channel, true); err != nil {
				t.Fatal(err)
			}
			if autoLockArmed(v) {
				t.Fatalf("%s must disarm the countdown", channel)
			}
			if autoLockClearCount(redis) == 0 {
				t.Fatal("expected the published deadline to be cleared")
			}
		})
	}
}

func TestHandlebarAutoLock_CancelWhenDisabledBySetting(t *testing.T) {
	v, _, redis := autoLockSystem(t, fsm.StateParked)
	if err := v.handleHandlebarPosition("handlebar_position", true); err != nil {
		t.Fatal(err)
	}
	if !autoLockArmed(v) {
		t.Fatal("precondition: expected armed")
	}

	redis.hashFields = map[string]string{"settings/" + handlebarAutoLockSettingKey: "0"}
	if err := v.handleSettingsUpdate(handlebarAutoLockSettingKey); err != nil {
		t.Fatal(err)
	}
	if autoLockArmed(v) {
		t.Fatal("disabling the setting must disarm the countdown")
	}
}

func TestHandlebarAutoLock_ExpiryLocks(t *testing.T) {
	v, io, redis := autoLockSystem(t, fsm.StateParked)
	// The handlebar is physically held at the detent when the countdown fires.
	io.setDigitalInput("handlebar_position", true)

	if err := v.handleHandlebarPosition("handlebar_position", true); err != nil {
		t.Fatal(err)
	}
	if !autoLockArmed(v) {
		t.Fatal("precondition: expected armed")
	}

	v.handlebarAutoLockExpired()

	waitForState(t, v, fsm.StateShuttingDown, 2*time.Second)
	if autoLockArmed(v) {
		t.Fatal("firing must disarm the countdown")
	}
	if autoLockClearCount(redis) == 0 {
		t.Fatal("expected the published deadline to be cleared")
	}
}

// The timer itself (not just the expiry handler) has to reach EvLock.
func TestHandlebarAutoLock_TimerFires(t *testing.T) {
	v, io, _ := autoLockSystem(t, fsm.StateParked)
	io.setDigitalInput("handlebar_position", true)
	v.mu.Lock()
	v.handlebarAutoLockSeconds = 1
	v.mu.Unlock()

	if err := v.handleHandlebarPosition("handlebar_position", true); err != nil {
		t.Fatal(err)
	}
	if !autoLockArmed(v) {
		t.Fatal("precondition: expected armed")
	}

	waitForState(t, v, fsm.StateShuttingDown, 3*time.Second)
}

// Leaving the parked family cancels an armed countdown.
func TestHandlebarAutoLock_CancelOnLeavingParked(t *testing.T) {
	v, _, redis := autoLockSystem(t, fsm.StateParked)
	if err := v.handleHandlebarPosition("handlebar_position", true); err != nil {
		t.Fatal(err)
	}
	if !autoLockArmed(v) {
		t.Fatal("precondition: expected armed")
	}

	if err := v.machine.SendSync(librefsm.Event{ID: fsm.EvLock}); err != nil {
		t.Fatal(err)
	}
	if autoLockArmed(v) {
		t.Fatal("leaving parked must disarm the countdown")
	}
	if autoLockClearCount(redis) == 0 {
		t.Fatal("expected the published deadline to be cleared")
	}
}
