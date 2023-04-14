#pragma once

#include "Arduino.h"
#include <vector>

using namespace std;

struct DataPoint {
    String name;
    float value;
};

class DataPacket {
    private:
        unsigned long timestamp;
        vector<DataPoint> data = {};
    public:
        DataPacket();
        DataPacket(vector<DataPoint> points);
        void add_data(String name, float value);
        void write_to_serial();
        void update_timestamp();
};