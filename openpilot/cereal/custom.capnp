using Cxx = import "/include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

# SCC-V ("slow for curves") state machine. Every field is a raw input or output of the machine in
# selfdrive/controls/lib/vision_turn_speed.py, so a route alone is enough to tune its thresholds.
# Published every cycle including while disabled: a gap in the trace would be ambiguous between
# "feature off" and "plannerd stalled".
struct SmartCruiseControlVisionState @0x81c2f05a394cf4af {
  state @0 :State;
  currentLatAcc @1 :Float32;   # m/s^2, v_ego^2 * curvature -- drives turning/leaving/finish
  maxPredLatAcc @2 :Float32;   # m/s^2, 97th pct of the model's predicted lat accel -- drives entering
  vTarget @3 :Float32;         # m/s, speed the curve solution asks for
  aTarget @4 :Float32;         # m/s^2, accel the state machine commands

  # mirrors VisionState in selfdrive/controls/lib/vision_turn_speed.py, ordinals included
  enum State {
    disabled @0;
    enabled @1;
    entering @2;
    turning @3;
    leaving @4;
    overriding @5;
  }
}

# H6 MK4 MADS-lite state (selfdrive/selfdrived/mads_h6.py): ACC and steering are independent, so a
# route needs to say which one is live and why.
struct MadsState @0xaedffd8f31e7b55d {
  longEnabled @0 :Bool;             # is ACC live, as MADS-lite sees it
  overrideSource @1 :OverrideSource;
  mismatch @2 :Bool;                # engaged with no panda allowing controls
  mismatchFrames @3 :UInt16;        # consecutive frames of that disagreement

  # Why longitudinal is overridden. The event raised for it is `gasPressedOverride`, reused for its
  # ET.OVERRIDE_LONGITUDINAL and empty alert -- without this field a brake-held override reads as
  # "gas pressed" in the log, with the driver's foot nowhere near the accelerator.
  enum OverrideSource {
    none @0;          # not overriding
    brake @1;         # brake or regen dropped ACC, steering kept
    lateralOnly @2;   # gentle stalk gesture engaged steering only, ACC never armed
  }
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
