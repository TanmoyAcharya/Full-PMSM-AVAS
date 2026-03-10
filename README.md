# Full-PMSM-AVAS
Using harmonic current injection in a PMSM traction motor to generate pedestrian alert sounds — no external speaker required
🎯 The Idea
Electric vehicles are dangerously quiet at low speeds. EU regulation ECE R138 requires every EV to emit a warning sound below 20 km/h via an Acoustic Vehicle Alert System (AVAS).
Standard solution: Mount a dedicated speaker + amplifier + enclosure.
This project: The PMSM traction motor already on the vehicle generates the alert sound by injecting a tiny sinusoidal component into the q-axis current. No extra hardware. Pure software.
Standard AVAS:   Controller ──► Separate Speaker ──► Sound
This Project:    Controller ──► q-axis Injection ──► Motor Vibration ──► Sound



📊 Simulation Results

50 kW Interior PMSM | 400V DC bus | Fixed-step solver (1e-4s) | 10s urban drive cycle


Fig 1 — Speed Profile & AVAS Activation Zone
Show Image
AVAS activates automatically below 20 km/h (pink zone). Active 100% of this drive cycle — correctly alerting pedestrians throughout the entire urban run.

Fig 2 — Speed-Proportional Tone Frequency
Show Image
Simulated frequency (scatter, coloured by time) tracks the target ramp curve exactly — 400 Hz at standstill rising to 1200 Hz at 20 km/h. ECE R138 minimum of 315 Hz respected throughout. Pedestrians hear the pitch rise as the car approaches.

Fig 3 — FFT of q-axis Current ⭐ Key Result
Show Image
This is the proof the injection works. At v ≈ 10 km/h the FFT reveals a razor-sharp peak at 801 Hz — matching the expected 800 Hz to within 1 Hz (FFT bin resolution). The spectrum is completely clean above 1 kHz with no spurious harmonics. This is the electrical fingerprint of the motor producing sound.

Fig 4 — Torque Ripple Analysis
Show Image
The tone injection adds a periodic ripple superimposed on the traction torque. The motor produces the alert while continuing to propel the vehicle normally.

Fig 5 — ECE R138 SPL Compliance Map
Show Image
2D compliance surface over injection amplitude vs vehicle speed. The design operating point (★ at 9A) lies within the legal SPL zone across all AVAS-active speeds.

🏗️ Simulink Model Architecture
5 subsystems, fully auto-wired by the build script:
[Speed Profile] ──► [Speed Controller] ──► i_q_traction ──► [Σ] ──► [FOC Controller]
                          ▲ wr                               ▲               │ Vd, Vq
                          │                                  │               ▼
[AVAS ToneGen] ◄── v_kmh ◄── [wr→km/h] ◄── [PMSM Motor] ◄──────────────────┘
      │ i_q_tone                               │ id,iq,we
      └────────────────────────────────────────┘
                                               │ iq_total
                                        [Acoustic Monitor]
                                             SPL [dB]
SubsystemDescriptionPMSM_MotorFull dq-axis ODE model — id, iq, wr dynamicsFOC_ControllerPI current loops + decoupling feedforwardSpeed_ControllerOuter PI speed loop → i_q_tractionAVAS_ToneGen⭐ Sinusoidal i_q injection, speed-proportional frequencyAcoustic_Monitor2nd-order TF: current → vibration → SPL in dB

🚀 Quick Start
matlab% Step 1: Build the fully-wired Simulink model (one time)
run('BUILD_FULL_PMSM_AVAS.m')

% Step 2: Simulate (fast settings)
set_param('PMSM_AVAS_Speaker','FixedStep','1e-4','StopTime','10');
out = sim('PMSM_AVAS_Speaker');

% Step 3: Generate all plots
run('matlab/plot_results.m')

⚖️ ECE R138 Compliance Summary
RequirementSpecThis DesignStatusActivation threshold< 20 km/h20 km/h✅Minimum SPL≥ 50 dB(A)~58 dB✅Maximum SPL≤ 75 dB(A)~58 dB✅Frequency range315–5000 Hz400–1200 Hz✅Speed variationRequiredPitch ∝ speed✅Cannot be disabled by driverMandatoryAuto only✅

💡 Hardware Savings vs Traditional AVAS
Traditional SpeakerThis ApproachExtra hardwareSpeaker + amp + housingNoneAdded weight~400 g0 gAdded cost~$50–200$0Failure pointsSpeaker, wire, ampNoneTorque penaltyNone< 1% of ratedSound profileFixed at manufactureFully reprogrammable

🔧 Motor Specs
ParameterValueRated Power50 kWRated Torque200 N·mMax Speed10,000 RPMDC Bus400 VPole Pairs4Rs0.01 ΩLd / Lq270 / 310 µHψf0.1546 WbAVAS Injection9 A (3% of I_q_max)

📋 Requirements

MATLAB R2021a+
Simulink
Control System Toolbox


📚 References

UN Regulation No. 138 — Acoustic Vehicle Alert Systems (UNECE, 2018)
Vas, P. — Sensorless Vector and Direct Torque Control, Oxford University Press
Mohan, N. — Electric Drives: An Integrative Approach
