#include "DataPacket.h"

DataPacket::DataPacket() {
    update_timestamp();
}

DataPacket::DataPacket(vector<DataPoint> points) {
    update_timestamp();
    for (DataPoint point: points) {
        data.push_back(point);
    }
}

void DataPacket::write_to_serial() {
    String output = "";
    output+= String(timestamp).c_str();
    for (DataPoint point: data) {
        output+=", ";
        output+= point.name.c_str();
        output+=", ";
        output+= String(point.value).c_str();
    }
    Serial.println(output);
}

void DataPacket::add_data(String name, float value) {
    DataPoint point;
    point.name = name;
    point.value = value;
    data.push_back(point);
}

void DataPacket::update_timestamp() {
    timestamp = millis();
}