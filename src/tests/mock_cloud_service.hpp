#pragma once

#include <gmock/gmock.h>
#include "../core/telematics/cloud_connector.hpp"

class MockCloudService {
public:
    MOCK_METHOD(void, setMockUpdate, (const std::string& version, const std::string& description));
    MOCK_METHOD(bool, connect, (const std::string& endpoint, const std::string& apiKey));
    MOCK_METHOD(bool, publishData, (const std::string& topic, const autocore::telematics::TelemetryData& data));
    MOCK_METHOD(std::vector<autocore::telematics::Message>, receiveMessages, ());
}; 