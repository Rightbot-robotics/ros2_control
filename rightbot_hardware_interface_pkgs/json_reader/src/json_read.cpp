//
// Created by inderjeet on 24/02/22.
//

#include <json_reader/json_read.h>

JsonRead::JsonRead(const std::string &file_path):file_path_(file_path) {
}

JsonRead::~JsonRead() {

}
bool JsonRead::parse() {
    Json::Reader reader;
    Json::Value value;
    std::filebuf fb;
    bool parsing_status = false;

    if (fb.open(file_path_, std::ios::in)) {
        std::istream stream(&fb);
        parsing_status = reader.parse(stream, value, false);
    }

    if (!parsing_status) {
//        logger_->error("Failed to parse config json. Error: {} ", reader.getFormattedErrorMessages());
        return parsing_status;
    }

//    logger_->debug("config parsing successful");

    // Todo: Add try catch throw here
    value_ = value;
    return parsing_status;

}

bool JsonRead::getElement(const std::string &base_element, Json::Value &value) {

    bool result = false;
    if (value_.isMember(base_element)) {
        value = value_[base_element];
        result = true;
    } else {
        value = Json::Value();
        result = false;
    }
    return result;
}
bool JsonRead::getValue(Json::Value &value) {
    value = value_;
    return true;
}
