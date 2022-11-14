#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/tuple/to_seq.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

#include <message_conversion_json/message_parser.hpp>

// Structure used to store the type support for a single interface type
using TypeSupport = rosidl_message_type_support_t;
using TypeInfo = rosidl_typesupport_introspection_cpp::MessageMembers;
using MemberInfo = rosidl_typesupport_introspection_cpp::MessageMember;

typedef const rosidl_message_type_support_t* (*get_message_ts_func)();

namespace message_conversion_json {

template <int RosTypeId>
struct TypeMappingCpp {};

#define TYPE_MAPPINGS                                                            \
  (rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT, float),                 \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE, double),           \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE, long double), \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR, signed char),        \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR, uint16_t),          \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN, bool),            \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET, uint8_t),           \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8, uint8_t),           \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8, int8_t),             \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16, uint16_t),         \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16, int16_t),           \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32, uint32_t),         \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32, int32_t),           \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64, uint64_t),         \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64, int64_t),           \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING, std::string),      \
      (rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING, std::u16string)

#define MAKE_TYPE_MAPPING_STRUCT(r, data, elem)         \
  template <>                                           \
  struct TypeMappingCpp<BOOST_PP_TUPLE_ELEM(0, elem)> { \
    using CppType = BOOST_PP_TUPLE_ELEM(1, elem);       \
    using SequenceType = std::vector<CppType>;          \
  };

#define MAKE_TYPE_MAPPINGS(seq) \
  BOOST_PP_SEQ_FOR_EACH(MAKE_TYPE_MAPPING_STRUCT, ~, BOOST_PP_TUPLE_TO_SEQ((seq)))

MAKE_TYPE_MAPPINGS(TYPE_MAPPINGS)

void toRosMsgImpl(const nlohmann::json& doc, const TypeInfo* typeinfo, uint8_t* buffer);

// Write an individual member into the binary message - generic
template <int RosTypeId>
void write_member_item(const nlohmann::json& doc, uint8_t* buffer) {
  using CppType = typename TypeMappingCpp<RosTypeId>::CppType;
  *reinterpret_cast<CppType*>(buffer) = doc.get<CppType>();
}

// Write an individual sequence member into the binary message - generic
template <int RosTypeId>
void write_sequence_member_item(const nlohmann::json& doc, uint8_t* buffer) {
  using CppType = typename TypeMappingCpp<RosTypeId>::CppType;
  using SequenceType = typename TypeMappingCpp<RosTypeId>::SequenceType;
  auto seq = reinterpret_cast<SequenceType*>(buffer);
  seq->push_back(doc.get<CppType>());
}

template <int RosTypeId>
void write_member_sequence(const nlohmann::json& doc, uint8_t* buffer, const MemberInfo& member) {
  if (member.is_upper_bound_ && doc.size() > member.array_size_) {
    throw std::runtime_error("doc sequence is more than capacity");
  }
  for (size_t i = 0; i < doc.size(); i++) {
    write_sequence_member_item<RosTypeId>(doc[i], buffer);
  }
}

// std::vector<bool> is different
// https://en.cppreference.com/w/cpp/container/vector_bool
template <>
void write_member_sequence<rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL>(
    const nlohmann::json& doc, uint8_t* buffer, const MemberInfo& member) {
  if (member.is_upper_bound_ && doc.size() > member.array_size_) {
    throw std::runtime_error("doc sequence is more than capacity");
  }
  // Just cast the sequence to std::vector<bool> and copy YAML node elements into it
  using SequenceType =
      typename TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL>::SequenceType;
  auto seq = reinterpret_cast<SequenceType*>(buffer);
  for (size_t i = 0; i < doc.size(); i++) {
    seq->push_back(doc[i].get<bool>());
  }
}

// Check if a field is a sequence of some kind
inline bool is_sequence(const MemberInfo& member) {
  return ((member.is_array_ && member.array_size_ == 0) || member.is_upper_bound_);
}

template <int RosTypeId>
void write_member(const nlohmann::json& doc, uint8_t* buffer, const MemberInfo& member) {
  using CppType = typename TypeMappingCpp<RosTypeId>::CppType;
  // Arrays and sequences have different struct representation. An array is represented by a
  // classic C array (pointer with data size == sizeof(type) * array_size).
  //
  // Sequences on the other hand use a custom-defined struct with data, size and capacity members.

  // Handle sequences
  if (is_sequence(member)) {
    write_member_sequence<RosTypeId>(doc[member.name_], buffer + member.offset_, member);
    return;
  }

  // Handle classic C arrays
  if (member.is_array_) {
    for (size_t i = 0; i < member.array_size_; i++) {
      write_member_item<RosTypeId>(doc[member.name_][i],
                                   buffer + member.offset_ + sizeof(CppType) * i);
    }
  } else {
    // Handle single-item members
    write_member_item<RosTypeId>(doc[member.name_], buffer + member.offset_);
  }
}

void write_member_sequence_nested(const nlohmann::json& doc, uint8_t* buffer,
                                  const MemberInfo& member) {
  if (member.is_upper_bound_ && doc.size() > member.array_size_) {
    throw std::runtime_error("doc sequence is more than capacity");
  }
  const TypeInfo* member_typeinfo = reinterpret_cast<const TypeInfo*>(member.members_->data);
  auto& seq = buffer;
  member.resize_function(seq, doc.size());
  for (size_t i = 0; i < doc.size(); i++) {
    toRosMsgImpl(doc[i], member_typeinfo, reinterpret_cast<uint8_t*>(member.get_function(seq, i)));
  }
}

void write_member_nested(const nlohmann::json& doc, uint8_t* buffer, const MemberInfo& member) {
  if (is_sequence(member)) {
    write_member_sequence_nested(doc[member.name_], buffer + member.offset_, member);
    return;
  }

  const TypeInfo* member_typeinfo = reinterpret_cast<const TypeInfo*>(member.members_->data);
  if (member.is_array_) {
    for (size_t i = 0; i < doc[member.name_].size(); i++) {
      toRosMsgImpl(doc[member.name_][i], member_typeinfo,
                   buffer + member.offset_ + member_typeinfo->size_of_ * i);
    }
  } else {
    toRosMsgImpl(doc[member.name_], member_typeinfo, buffer + member.offset_);
  }
}

void toRosMsgImpl(const nlohmann::json& doc, const TypeInfo* typeinfo, uint8_t* buffer) {
  for (uint32_t i = 0; i < typeinfo->member_count_; i++) {
    const auto& member = typeinfo->members_[i];

    if (!doc.contains(member.name_)) {
      continue;
    }

    switch (member.type_id_) {
      using namespace rosidl_typesupport_introspection_cpp;
      case ROS_TYPE_FLOAT:
        write_member<ROS_TYPE_FLOAT>(doc, buffer, member);
        break;
      case ROS_TYPE_DOUBLE:
        write_member<ROS_TYPE_DOUBLE>(doc, buffer, member);
        break;
      case ROS_TYPE_LONG_DOUBLE:
        write_member<ROS_TYPE_LONG_DOUBLE>(doc, buffer, member);
        break;
      case ROS_TYPE_CHAR:
        write_member<ROS_TYPE_CHAR>(doc, buffer, member);
        break;
      case ROS_TYPE_WCHAR:
        write_member<ROS_TYPE_WCHAR>(doc, buffer, member);
        break;
      case ROS_TYPE_BOOLEAN:
        write_member<ROS_TYPE_BOOLEAN>(doc, buffer, member);
        break;
      case ROS_TYPE_OCTET:
        write_member<ROS_TYPE_OCTET>(doc, buffer, member);
        break;
      case ROS_TYPE_UINT8:
        write_member<ROS_TYPE_UINT8>(doc, buffer, member);
        break;
      case ROS_TYPE_INT8:
        write_member<ROS_TYPE_INT8>(doc, buffer, member);
        break;
      case ROS_TYPE_UINT16:
        write_member<ROS_TYPE_UINT16>(doc, buffer, member);
        break;
      case ROS_TYPE_INT16:
        write_member<ROS_TYPE_INT16>(doc, buffer, member);
        break;
      case ROS_TYPE_UINT32:
        write_member<ROS_TYPE_UINT32>(doc, buffer, member);
        break;
      case ROS_TYPE_INT32:
        write_member<ROS_TYPE_INT32>(doc, buffer, member);
        break;
      case ROS_TYPE_UINT64:
        write_member<ROS_TYPE_UINT64>(doc, buffer, member);
        break;
      case ROS_TYPE_INT64:
        write_member<ROS_TYPE_INT64>(doc, buffer, member);
        break;
      case ROS_TYPE_STRING:
        write_member<ROS_TYPE_STRING>(doc, buffer, member);
        break;
      case ROS_TYPE_WSTRING:
        write_member<ROS_TYPE_WSTRING>(doc, buffer, member);
        break;
      case ROS_TYPE_MESSAGE:
        write_member_nested(doc, buffer, member);
        break;
      default:
        throw std::runtime_error("unknown type");
    }
  }
}

const TypeInfo* getTypeInfo(const std::string& interface_type,
                            std::shared_ptr<rcpputils::SharedLibrary> ts_lib) {
  const auto pos = interface_type.find_first_of('/');
  if (pos == std::string::npos || pos >= interface_type.size()) {
    throw std::runtime_error("Invalid data type");
  }

  const std::string pkg_name = interface_type.substr(0, pos);
  const std::string msg_name = interface_type.substr(pos + 1);

  auto typesupport_library = ts_lib ? ts_lib
                                    : rclcpp::get_typesupport_library(
                                          interface_type, "rosidl_typesupport_introspection_cpp");

  const std::string ts_func_name =
      "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + pkg_name +
      "__msg__" + msg_name;

  get_message_ts_func introspection_type_support_handle_func =
      reinterpret_cast<get_message_ts_func>(typesupport_library->get_symbol(ts_func_name.c_str()));
  if (nullptr == introspection_type_support_handle_func) {
    return nullptr;
  }

  // Call the function to get the introspection information we want
  const rosidl_message_type_support_t* introspection_ts = introspection_type_support_handle_func();
  return reinterpret_cast<const TypeInfo*>(introspection_ts->data);
}

RosMessage toRosMsg(const TypeInfo* type_info, const std::string& json,
                    rcutils_allocator_t* allocator) {
  if (nullptr == type_info) {
    throw std::runtime_error("Type info not available");
  }

  RosMessage msg = {
      type_info, static_cast<uint8_t*>(allocator->allocate(type_info->size_of_, allocator->state))};

  type_info->init_function(msg.data, rosidl_runtime_cpp::MessageInitialization::ALL);

  const auto json_data = nlohmann::json::parse(json);
  toRosMsgImpl(json_data, type_info, msg.data);
  return msg;
}

}  // namespace message_conversion_json