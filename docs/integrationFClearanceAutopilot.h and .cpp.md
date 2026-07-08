// integration/FClearanceAutopilot.h
#pragma once

extern "C" {
#include "autopilot.h" // generated header, adjust path as needed
}

class FClearanceAutopilot
{
public:
    FClearanceAutopilot();
    ~FClearanceAutopilot();

    // Set commanded references (radians, meters, m/s)
    void SetCommand(float PsiCmdRad, float HCmdM, float VCmdMps);

    // Update measured states (radians, meters, m/s, rad/s)
    void UpdateMeasurements(float Phi, float Theta, float Psi, float H, float V, float p, float q);

    // Step the autopilot (call at CODEGEN_STEP frequency)
    void Step();

    // Get actuator commands
    float GetAileron() const;
    float GetElevator() const;
    float GetThrottle() const;

private:
    // storage for inputs/outputs following generated API (example names)
    Autopilot_U rtU;
    Autopilot_Y rtY;

    // internal time accumulator (if needed)
    double Accumulator;
};


// integration/FClearanceAutopilot.cpp
#include "FClearanceAutopilot.h"
#include <cstring>

FClearanceAutopilot::FClearanceAutopilot()
{
    memset(&rtU, 0, sizeof(rtU));
    memset(&rtY, 0, sizeof(rtY));
    Accumulator = 0.0;
    Autopilot_initialize(); // if generated init function exists
}

FClearanceAutopilot::~FClearanceAutopilot()
{
    Autopilot_terminate(); // if exists
}

void FClearanceAutopilot::SetCommand(float PsiCmdRad, float HCmdM, float VCmdMps)
{
    rtU.psi_cmd = PsiCmdRad;
    rtU.h_cmd   = HCmdM;
    rtU.V_cmd   = VCmdMps;
}

void FClearanceAutopilot::UpdateMeasurements(float Phi, float Theta, float Psi, float H, float V, float p, float q)
{
    rtU.phi   = Phi;
    rtU.theta = Theta;
    rtU.psi   = Psi;
    rtU.h     = H;
    rtU.V     = V;
    rtU.p     = p;
    rtU.q     = q;
}

void FClearanceAutopilot::Step()
{
    Autopilot_step(); // generated step routine that uses rtU and fills rtY
    // assume rtY is populated; otherwise call generated API appropriately
}

float FClearanceAutopilot::GetAileron() const
{
    return rtY.delta_a;
}
float FClearanceAutopilot::GetElevator() const
{
    return rtY.delta_e;
}
float FClearanceAutopilot::GetThrottle() const
{
    return rtY.delta_t;
}
