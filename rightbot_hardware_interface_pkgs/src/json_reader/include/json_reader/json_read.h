//
// Created by inderjeet on 24/02/22.
//

#ifndef JSON_READ_H_
#define JSON_READ_H_

#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>


class JsonRead {
 public:
    JsonRead(const std::string &file_path);
    ~JsonRead();

 private:
    std::string file_path_;
    Json::Value value_;

 public:
    bool parse();
    bool getElement(const std::string& base_element, Json::Value& value);
    bool getValue(Json::Value &value);

};
#endif //JSON_READ_H_
