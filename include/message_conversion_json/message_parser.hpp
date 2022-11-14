#pragma once

#include <memory>
#include <string>
#include <utility>

#include <rcutils/allocator.h>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace rcpputils {
class SharedLibrary;
}

namespace message_conversion_json {

using TypeInfo = rosidl_typesupport_introspection_cpp::MessageMembers;

struct RosMessage {
  const TypeInfo* type_info;
  uint8_t* data;
};

const TypeInfo* getTypeInfo(const std::string& interface_type,
                            std::shared_ptr<rcpputils::SharedLibrary> ts_lib = nullptr);

RosMessage toRosMsg(const TypeInfo* type_info, const std::string& json,
                    rcutils_allocator_t* allocator);

}  // namespace message_conversion_json