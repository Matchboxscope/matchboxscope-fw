#pragma once

struct ModuleConfig
{
    bool analog = false;
    bool dac = false;
    bool digital = true;
    bool laser = true;
    bool led = true;
    bool motor = true;
    bool pid = false;
    bool scanner = false;
    bool sensor = false;
    bool slm = false;
    bool home = true;
};
