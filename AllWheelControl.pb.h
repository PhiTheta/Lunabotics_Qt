// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: AllWheelControl.proto

#ifndef PROTOBUF_AllWheelControl_2eproto__INCLUDED
#define PROTOBUF_AllWheelControl_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace lunabotics {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_AllWheelControl_2eproto();
void protobuf_AssignDesc_AllWheelControl_2eproto();
void protobuf_ShutdownFile_AllWheelControl_2eproto();

class AllWheelControl;
class AllWheelControl_Wheels;
class AllWheelControl_ExplicitControl;
class AllWheelControl_PredefinedControl;

enum AllWheelControl_AllWheelControlType {
  AllWheelControl_AllWheelControlType_EXPLICIT = 1,
  AllWheelControl_AllWheelControlType_PREDEFINED = 2
};
bool AllWheelControl_AllWheelControlType_IsValid(int value);
const AllWheelControl_AllWheelControlType AllWheelControl_AllWheelControlType_AllWheelControlType_MIN = AllWheelControl_AllWheelControlType_EXPLICIT;
const AllWheelControl_AllWheelControlType AllWheelControl_AllWheelControlType_AllWheelControlType_MAX = AllWheelControl_AllWheelControlType_PREDEFINED;
const int AllWheelControl_AllWheelControlType_AllWheelControlType_ARRAYSIZE = AllWheelControl_AllWheelControlType_AllWheelControlType_MAX + 1;

const ::google::protobuf::EnumDescriptor* AllWheelControl_AllWheelControlType_descriptor();
inline const ::std::string& AllWheelControl_AllWheelControlType_Name(AllWheelControl_AllWheelControlType value) {
  return ::google::protobuf::internal::NameOfEnum(
    AllWheelControl_AllWheelControlType_descriptor(), value);
}
inline bool AllWheelControl_AllWheelControlType_Parse(
    const ::std::string& name, AllWheelControl_AllWheelControlType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<AllWheelControl_AllWheelControlType>(
    AllWheelControl_AllWheelControlType_descriptor(), name, value);
}
enum AllWheelControl_PredefinedControlType {
  AllWheelControl_PredefinedControlType_CRAB_LEFT = 1,
  AllWheelControl_PredefinedControlType_CRAB_RIGHT = 2,
  AllWheelControl_PredefinedControlType_WHEELS_INWARD = 3,
  AllWheelControl_PredefinedControlType_WHEELS_OUTWARD = 4
};
bool AllWheelControl_PredefinedControlType_IsValid(int value);
const AllWheelControl_PredefinedControlType AllWheelControl_PredefinedControlType_PredefinedControlType_MIN = AllWheelControl_PredefinedControlType_CRAB_LEFT;
const AllWheelControl_PredefinedControlType AllWheelControl_PredefinedControlType_PredefinedControlType_MAX = AllWheelControl_PredefinedControlType_WHEELS_OUTWARD;
const int AllWheelControl_PredefinedControlType_PredefinedControlType_ARRAYSIZE = AllWheelControl_PredefinedControlType_PredefinedControlType_MAX + 1;

const ::google::protobuf::EnumDescriptor* AllWheelControl_PredefinedControlType_descriptor();
inline const ::std::string& AllWheelControl_PredefinedControlType_Name(AllWheelControl_PredefinedControlType value) {
  return ::google::protobuf::internal::NameOfEnum(
    AllWheelControl_PredefinedControlType_descriptor(), value);
}
inline bool AllWheelControl_PredefinedControlType_Parse(
    const ::std::string& name, AllWheelControl_PredefinedControlType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<AllWheelControl_PredefinedControlType>(
    AllWheelControl_PredefinedControlType_descriptor(), name, value);
}
// ===================================================================

class AllWheelControl_Wheels : public ::google::protobuf::Message {
 public:
  AllWheelControl_Wheels();
  virtual ~AllWheelControl_Wheels();

  AllWheelControl_Wheels(const AllWheelControl_Wheels& from);

  inline AllWheelControl_Wheels& operator=(const AllWheelControl_Wheels& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const AllWheelControl_Wheels& default_instance();

  void Swap(AllWheelControl_Wheels* other);

  // implements Message ----------------------------------------------

  AllWheelControl_Wheels* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const AllWheelControl_Wheels& from);
  void MergeFrom(const AllWheelControl_Wheels& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required float left_front = 1;
  inline bool has_left_front() const;
  inline void clear_left_front();
  static const int kLeftFrontFieldNumber = 1;
  inline float left_front() const;
  inline void set_left_front(float value);

  // required float right_front = 2;
  inline bool has_right_front() const;
  inline void clear_right_front();
  static const int kRightFrontFieldNumber = 2;
  inline float right_front() const;
  inline void set_right_front(float value);

  // required float left_rear = 3;
  inline bool has_left_rear() const;
  inline void clear_left_rear();
  static const int kLeftRearFieldNumber = 3;
  inline float left_rear() const;
  inline void set_left_rear(float value);

  // required float right_rear = 4;
  inline bool has_right_rear() const;
  inline void clear_right_rear();
  static const int kRightRearFieldNumber = 4;
  inline float right_rear() const;
  inline void set_right_rear(float value);

  // @@protoc_insertion_point(class_scope:lunabotics.AllWheelControl.Wheels)
 private:
  inline void set_has_left_front();
  inline void clear_has_left_front();
  inline void set_has_right_front();
  inline void clear_has_right_front();
  inline void set_has_left_rear();
  inline void clear_has_left_rear();
  inline void set_has_right_rear();
  inline void clear_has_right_rear();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  float left_front_;
  float right_front_;
  float left_rear_;
  float right_rear_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(4 + 31) / 32];

  friend void  protobuf_AddDesc_AllWheelControl_2eproto();
  friend void protobuf_AssignDesc_AllWheelControl_2eproto();
  friend void protobuf_ShutdownFile_AllWheelControl_2eproto();

  void InitAsDefaultInstance();
  static AllWheelControl_Wheels* default_instance_;
};
// -------------------------------------------------------------------

class AllWheelControl_ExplicitControl : public ::google::protobuf::Message {
 public:
  AllWheelControl_ExplicitControl();
  virtual ~AllWheelControl_ExplicitControl();

  AllWheelControl_ExplicitControl(const AllWheelControl_ExplicitControl& from);

  inline AllWheelControl_ExplicitControl& operator=(const AllWheelControl_ExplicitControl& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const AllWheelControl_ExplicitControl& default_instance();

  void Swap(AllWheelControl_ExplicitControl* other);

  // implements Message ----------------------------------------------

  AllWheelControl_ExplicitControl* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const AllWheelControl_ExplicitControl& from);
  void MergeFrom(const AllWheelControl_ExplicitControl& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required .lunabotics.AllWheelControl.Wheels steering = 1;
  inline bool has_steering() const;
  inline void clear_steering();
  static const int kSteeringFieldNumber = 1;
  inline const ::lunabotics::AllWheelControl_Wheels& steering() const;
  inline ::lunabotics::AllWheelControl_Wheels* mutable_steering();
  inline ::lunabotics::AllWheelControl_Wheels* release_steering();
  inline void set_allocated_steering(::lunabotics::AllWheelControl_Wheels* steering);

  // required .lunabotics.AllWheelControl.Wheels driving = 2;
  inline bool has_driving() const;
  inline void clear_driving();
  static const int kDrivingFieldNumber = 2;
  inline const ::lunabotics::AllWheelControl_Wheels& driving() const;
  inline ::lunabotics::AllWheelControl_Wheels* mutable_driving();
  inline ::lunabotics::AllWheelControl_Wheels* release_driving();
  inline void set_allocated_driving(::lunabotics::AllWheelControl_Wheels* driving);

  // @@protoc_insertion_point(class_scope:lunabotics.AllWheelControl.ExplicitControl)
 private:
  inline void set_has_steering();
  inline void clear_has_steering();
  inline void set_has_driving();
  inline void clear_has_driving();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::lunabotics::AllWheelControl_Wheels* steering_;
  ::lunabotics::AllWheelControl_Wheels* driving_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(2 + 31) / 32];

  friend void  protobuf_AddDesc_AllWheelControl_2eproto();
  friend void protobuf_AssignDesc_AllWheelControl_2eproto();
  friend void protobuf_ShutdownFile_AllWheelControl_2eproto();

  void InitAsDefaultInstance();
  static AllWheelControl_ExplicitControl* default_instance_;
};
// -------------------------------------------------------------------

class AllWheelControl_PredefinedControl : public ::google::protobuf::Message {
 public:
  AllWheelControl_PredefinedControl();
  virtual ~AllWheelControl_PredefinedControl();

  AllWheelControl_PredefinedControl(const AllWheelControl_PredefinedControl& from);

  inline AllWheelControl_PredefinedControl& operator=(const AllWheelControl_PredefinedControl& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const AllWheelControl_PredefinedControl& default_instance();

  void Swap(AllWheelControl_PredefinedControl* other);

  // implements Message ----------------------------------------------

  AllWheelControl_PredefinedControl* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const AllWheelControl_PredefinedControl& from);
  void MergeFrom(const AllWheelControl_PredefinedControl& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required .lunabotics.AllWheelControl.PredefinedControlType command = 1;
  inline bool has_command() const;
  inline void clear_command();
  static const int kCommandFieldNumber = 1;
  inline ::lunabotics::AllWheelControl_PredefinedControlType command() const;
  inline void set_command(::lunabotics::AllWheelControl_PredefinedControlType value);

  // @@protoc_insertion_point(class_scope:lunabotics.AllWheelControl.PredefinedControl)
 private:
  inline void set_has_command();
  inline void clear_has_command();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  int command_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(1 + 31) / 32];

  friend void  protobuf_AddDesc_AllWheelControl_2eproto();
  friend void protobuf_AssignDesc_AllWheelControl_2eproto();
  friend void protobuf_ShutdownFile_AllWheelControl_2eproto();

  void InitAsDefaultInstance();
  static AllWheelControl_PredefinedControl* default_instance_;
};
// -------------------------------------------------------------------

class AllWheelControl : public ::google::protobuf::Message {
 public:
  AllWheelControl();
  virtual ~AllWheelControl();

  AllWheelControl(const AllWheelControl& from);

  inline AllWheelControl& operator=(const AllWheelControl& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const AllWheelControl& default_instance();

  void Swap(AllWheelControl* other);

  // implements Message ----------------------------------------------

  AllWheelControl* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const AllWheelControl& from);
  void MergeFrom(const AllWheelControl& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  typedef AllWheelControl_Wheels Wheels;
  typedef AllWheelControl_ExplicitControl ExplicitControl;
  typedef AllWheelControl_PredefinedControl PredefinedControl;

  typedef AllWheelControl_AllWheelControlType AllWheelControlType;
  static const AllWheelControlType EXPLICIT = AllWheelControl_AllWheelControlType_EXPLICIT;
  static const AllWheelControlType PREDEFINED = AllWheelControl_AllWheelControlType_PREDEFINED;
  static inline bool AllWheelControlType_IsValid(int value) {
    return AllWheelControl_AllWheelControlType_IsValid(value);
  }
  static const AllWheelControlType AllWheelControlType_MIN =
    AllWheelControl_AllWheelControlType_AllWheelControlType_MIN;
  static const AllWheelControlType AllWheelControlType_MAX =
    AllWheelControl_AllWheelControlType_AllWheelControlType_MAX;
  static const int AllWheelControlType_ARRAYSIZE =
    AllWheelControl_AllWheelControlType_AllWheelControlType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  AllWheelControlType_descriptor() {
    return AllWheelControl_AllWheelControlType_descriptor();
  }
  static inline const ::std::string& AllWheelControlType_Name(AllWheelControlType value) {
    return AllWheelControl_AllWheelControlType_Name(value);
  }
  static inline bool AllWheelControlType_Parse(const ::std::string& name,
      AllWheelControlType* value) {
    return AllWheelControl_AllWheelControlType_Parse(name, value);
  }

  typedef AllWheelControl_PredefinedControlType PredefinedControlType;
  static const PredefinedControlType CRAB_LEFT = AllWheelControl_PredefinedControlType_CRAB_LEFT;
  static const PredefinedControlType CRAB_RIGHT = AllWheelControl_PredefinedControlType_CRAB_RIGHT;
  static const PredefinedControlType WHEELS_INWARD = AllWheelControl_PredefinedControlType_WHEELS_INWARD;
  static const PredefinedControlType WHEELS_OUTWARD = AllWheelControl_PredefinedControlType_WHEELS_OUTWARD;
  static inline bool PredefinedControlType_IsValid(int value) {
    return AllWheelControl_PredefinedControlType_IsValid(value);
  }
  static const PredefinedControlType PredefinedControlType_MIN =
    AllWheelControl_PredefinedControlType_PredefinedControlType_MIN;
  static const PredefinedControlType PredefinedControlType_MAX =
    AllWheelControl_PredefinedControlType_PredefinedControlType_MAX;
  static const int PredefinedControlType_ARRAYSIZE =
    AllWheelControl_PredefinedControlType_PredefinedControlType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  PredefinedControlType_descriptor() {
    return AllWheelControl_PredefinedControlType_descriptor();
  }
  static inline const ::std::string& PredefinedControlType_Name(PredefinedControlType value) {
    return AllWheelControl_PredefinedControlType_Name(value);
  }
  static inline bool PredefinedControlType_Parse(const ::std::string& name,
      PredefinedControlType* value) {
    return AllWheelControl_PredefinedControlType_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // required .lunabotics.AllWheelControl.AllWheelControlType all_wheel_type = 1;
  inline bool has_all_wheel_type() const;
  inline void clear_all_wheel_type();
  static const int kAllWheelTypeFieldNumber = 1;
  inline ::lunabotics::AllWheelControl_AllWheelControlType all_wheel_type() const;
  inline void set_all_wheel_type(::lunabotics::AllWheelControl_AllWheelControlType value);

  // optional .lunabotics.AllWheelControl.ExplicitControl explicit_data = 2;
  inline bool has_explicit_data() const;
  inline void clear_explicit_data();
  static const int kExplicitDataFieldNumber = 2;
  inline const ::lunabotics::AllWheelControl_ExplicitControl& explicit_data() const;
  inline ::lunabotics::AllWheelControl_ExplicitControl* mutable_explicit_data();
  inline ::lunabotics::AllWheelControl_ExplicitControl* release_explicit_data();
  inline void set_allocated_explicit_data(::lunabotics::AllWheelControl_ExplicitControl* explicit_data);

  // optional .lunabotics.AllWheelControl.PredefinedControl predefined_data = 3;
  inline bool has_predefined_data() const;
  inline void clear_predefined_data();
  static const int kPredefinedDataFieldNumber = 3;
  inline const ::lunabotics::AllWheelControl_PredefinedControl& predefined_data() const;
  inline ::lunabotics::AllWheelControl_PredefinedControl* mutable_predefined_data();
  inline ::lunabotics::AllWheelControl_PredefinedControl* release_predefined_data();
  inline void set_allocated_predefined_data(::lunabotics::AllWheelControl_PredefinedControl* predefined_data);

  // @@protoc_insertion_point(class_scope:lunabotics.AllWheelControl)
 private:
  inline void set_has_all_wheel_type();
  inline void clear_has_all_wheel_type();
  inline void set_has_explicit_data();
  inline void clear_has_explicit_data();
  inline void set_has_predefined_data();
  inline void clear_has_predefined_data();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::lunabotics::AllWheelControl_ExplicitControl* explicit_data_;
  ::lunabotics::AllWheelControl_PredefinedControl* predefined_data_;
  int all_wheel_type_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];

  friend void  protobuf_AddDesc_AllWheelControl_2eproto();
  friend void protobuf_AssignDesc_AllWheelControl_2eproto();
  friend void protobuf_ShutdownFile_AllWheelControl_2eproto();

  void InitAsDefaultInstance();
  static AllWheelControl* default_instance_;
};
// ===================================================================


// ===================================================================

// AllWheelControl_Wheels

// required float left_front = 1;
inline bool AllWheelControl_Wheels::has_left_front() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void AllWheelControl_Wheels::set_has_left_front() {
  _has_bits_[0] |= 0x00000001u;
}
inline void AllWheelControl_Wheels::clear_has_left_front() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void AllWheelControl_Wheels::clear_left_front() {
  left_front_ = 0;
  clear_has_left_front();
}
inline float AllWheelControl_Wheels::left_front() const {
  return left_front_;
}
inline void AllWheelControl_Wheels::set_left_front(float value) {
  set_has_left_front();
  left_front_ = value;
}

// required float right_front = 2;
inline bool AllWheelControl_Wheels::has_right_front() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void AllWheelControl_Wheels::set_has_right_front() {
  _has_bits_[0] |= 0x00000002u;
}
inline void AllWheelControl_Wheels::clear_has_right_front() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void AllWheelControl_Wheels::clear_right_front() {
  right_front_ = 0;
  clear_has_right_front();
}
inline float AllWheelControl_Wheels::right_front() const {
  return right_front_;
}
inline void AllWheelControl_Wheels::set_right_front(float value) {
  set_has_right_front();
  right_front_ = value;
}

// required float left_rear = 3;
inline bool AllWheelControl_Wheels::has_left_rear() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void AllWheelControl_Wheels::set_has_left_rear() {
  _has_bits_[0] |= 0x00000004u;
}
inline void AllWheelControl_Wheels::clear_has_left_rear() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void AllWheelControl_Wheels::clear_left_rear() {
  left_rear_ = 0;
  clear_has_left_rear();
}
inline float AllWheelControl_Wheels::left_rear() const {
  return left_rear_;
}
inline void AllWheelControl_Wheels::set_left_rear(float value) {
  set_has_left_rear();
  left_rear_ = value;
}

// required float right_rear = 4;
inline bool AllWheelControl_Wheels::has_right_rear() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void AllWheelControl_Wheels::set_has_right_rear() {
  _has_bits_[0] |= 0x00000008u;
}
inline void AllWheelControl_Wheels::clear_has_right_rear() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void AllWheelControl_Wheels::clear_right_rear() {
  right_rear_ = 0;
  clear_has_right_rear();
}
inline float AllWheelControl_Wheels::right_rear() const {
  return right_rear_;
}
inline void AllWheelControl_Wheels::set_right_rear(float value) {
  set_has_right_rear();
  right_rear_ = value;
}

// -------------------------------------------------------------------

// AllWheelControl_ExplicitControl

// required .lunabotics.AllWheelControl.Wheels steering = 1;
inline bool AllWheelControl_ExplicitControl::has_steering() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void AllWheelControl_ExplicitControl::set_has_steering() {
  _has_bits_[0] |= 0x00000001u;
}
inline void AllWheelControl_ExplicitControl::clear_has_steering() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void AllWheelControl_ExplicitControl::clear_steering() {
  if (steering_ != NULL) steering_->::lunabotics::AllWheelControl_Wheels::Clear();
  clear_has_steering();
}
inline const ::lunabotics::AllWheelControl_Wheels& AllWheelControl_ExplicitControl::steering() const {
  return steering_ != NULL ? *steering_ : *default_instance_->steering_;
}
inline ::lunabotics::AllWheelControl_Wheels* AllWheelControl_ExplicitControl::mutable_steering() {
  set_has_steering();
  if (steering_ == NULL) steering_ = new ::lunabotics::AllWheelControl_Wheels;
  return steering_;
}
inline ::lunabotics::AllWheelControl_Wheels* AllWheelControl_ExplicitControl::release_steering() {
  clear_has_steering();
  ::lunabotics::AllWheelControl_Wheels* temp = steering_;
  steering_ = NULL;
  return temp;
}
inline void AllWheelControl_ExplicitControl::set_allocated_steering(::lunabotics::AllWheelControl_Wheels* steering) {
  delete steering_;
  steering_ = steering;
  if (steering) {
    set_has_steering();
  } else {
    clear_has_steering();
  }
}

// required .lunabotics.AllWheelControl.Wheels driving = 2;
inline bool AllWheelControl_ExplicitControl::has_driving() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void AllWheelControl_ExplicitControl::set_has_driving() {
  _has_bits_[0] |= 0x00000002u;
}
inline void AllWheelControl_ExplicitControl::clear_has_driving() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void AllWheelControl_ExplicitControl::clear_driving() {
  if (driving_ != NULL) driving_->::lunabotics::AllWheelControl_Wheels::Clear();
  clear_has_driving();
}
inline const ::lunabotics::AllWheelControl_Wheels& AllWheelControl_ExplicitControl::driving() const {
  return driving_ != NULL ? *driving_ : *default_instance_->driving_;
}
inline ::lunabotics::AllWheelControl_Wheels* AllWheelControl_ExplicitControl::mutable_driving() {
  set_has_driving();
  if (driving_ == NULL) driving_ = new ::lunabotics::AllWheelControl_Wheels;
  return driving_;
}
inline ::lunabotics::AllWheelControl_Wheels* AllWheelControl_ExplicitControl::release_driving() {
  clear_has_driving();
  ::lunabotics::AllWheelControl_Wheels* temp = driving_;
  driving_ = NULL;
  return temp;
}
inline void AllWheelControl_ExplicitControl::set_allocated_driving(::lunabotics::AllWheelControl_Wheels* driving) {
  delete driving_;
  driving_ = driving;
  if (driving) {
    set_has_driving();
  } else {
    clear_has_driving();
  }
}

// -------------------------------------------------------------------

// AllWheelControl_PredefinedControl

// required .lunabotics.AllWheelControl.PredefinedControlType command = 1;
inline bool AllWheelControl_PredefinedControl::has_command() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void AllWheelControl_PredefinedControl::set_has_command() {
  _has_bits_[0] |= 0x00000001u;
}
inline void AllWheelControl_PredefinedControl::clear_has_command() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void AllWheelControl_PredefinedControl::clear_command() {
  command_ = 1;
  clear_has_command();
}
inline ::lunabotics::AllWheelControl_PredefinedControlType AllWheelControl_PredefinedControl::command() const {
  return static_cast< ::lunabotics::AllWheelControl_PredefinedControlType >(command_);
}
inline void AllWheelControl_PredefinedControl::set_command(::lunabotics::AllWheelControl_PredefinedControlType value) {
  assert(::lunabotics::AllWheelControl_PredefinedControlType_IsValid(value));
  set_has_command();
  command_ = value;
}

// -------------------------------------------------------------------

// AllWheelControl

// required .lunabotics.AllWheelControl.AllWheelControlType all_wheel_type = 1;
inline bool AllWheelControl::has_all_wheel_type() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void AllWheelControl::set_has_all_wheel_type() {
  _has_bits_[0] |= 0x00000001u;
}
inline void AllWheelControl::clear_has_all_wheel_type() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void AllWheelControl::clear_all_wheel_type() {
  all_wheel_type_ = 1;
  clear_has_all_wheel_type();
}
inline ::lunabotics::AllWheelControl_AllWheelControlType AllWheelControl::all_wheel_type() const {
  return static_cast< ::lunabotics::AllWheelControl_AllWheelControlType >(all_wheel_type_);
}
inline void AllWheelControl::set_all_wheel_type(::lunabotics::AllWheelControl_AllWheelControlType value) {
  assert(::lunabotics::AllWheelControl_AllWheelControlType_IsValid(value));
  set_has_all_wheel_type();
  all_wheel_type_ = value;
}

// optional .lunabotics.AllWheelControl.ExplicitControl explicit_data = 2;
inline bool AllWheelControl::has_explicit_data() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void AllWheelControl::set_has_explicit_data() {
  _has_bits_[0] |= 0x00000002u;
}
inline void AllWheelControl::clear_has_explicit_data() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void AllWheelControl::clear_explicit_data() {
  if (explicit_data_ != NULL) explicit_data_->::lunabotics::AllWheelControl_ExplicitControl::Clear();
  clear_has_explicit_data();
}
inline const ::lunabotics::AllWheelControl_ExplicitControl& AllWheelControl::explicit_data() const {
  return explicit_data_ != NULL ? *explicit_data_ : *default_instance_->explicit_data_;
}
inline ::lunabotics::AllWheelControl_ExplicitControl* AllWheelControl::mutable_explicit_data() {
  set_has_explicit_data();
  if (explicit_data_ == NULL) explicit_data_ = new ::lunabotics::AllWheelControl_ExplicitControl;
  return explicit_data_;
}
inline ::lunabotics::AllWheelControl_ExplicitControl* AllWheelControl::release_explicit_data() {
  clear_has_explicit_data();
  ::lunabotics::AllWheelControl_ExplicitControl* temp = explicit_data_;
  explicit_data_ = NULL;
  return temp;
}
inline void AllWheelControl::set_allocated_explicit_data(::lunabotics::AllWheelControl_ExplicitControl* explicit_data) {
  delete explicit_data_;
  explicit_data_ = explicit_data;
  if (explicit_data) {
    set_has_explicit_data();
  } else {
    clear_has_explicit_data();
  }
}

// optional .lunabotics.AllWheelControl.PredefinedControl predefined_data = 3;
inline bool AllWheelControl::has_predefined_data() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void AllWheelControl::set_has_predefined_data() {
  _has_bits_[0] |= 0x00000004u;
}
inline void AllWheelControl::clear_has_predefined_data() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void AllWheelControl::clear_predefined_data() {
  if (predefined_data_ != NULL) predefined_data_->::lunabotics::AllWheelControl_PredefinedControl::Clear();
  clear_has_predefined_data();
}
inline const ::lunabotics::AllWheelControl_PredefinedControl& AllWheelControl::predefined_data() const {
  return predefined_data_ != NULL ? *predefined_data_ : *default_instance_->predefined_data_;
}
inline ::lunabotics::AllWheelControl_PredefinedControl* AllWheelControl::mutable_predefined_data() {
  set_has_predefined_data();
  if (predefined_data_ == NULL) predefined_data_ = new ::lunabotics::AllWheelControl_PredefinedControl;
  return predefined_data_;
}
inline ::lunabotics::AllWheelControl_PredefinedControl* AllWheelControl::release_predefined_data() {
  clear_has_predefined_data();
  ::lunabotics::AllWheelControl_PredefinedControl* temp = predefined_data_;
  predefined_data_ = NULL;
  return temp;
}
inline void AllWheelControl::set_allocated_predefined_data(::lunabotics::AllWheelControl_PredefinedControl* predefined_data) {
  delete predefined_data_;
  predefined_data_ = predefined_data;
  if (predefined_data) {
    set_has_predefined_data();
  } else {
    clear_has_predefined_data();
  }
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace lunabotics

#ifndef SWIG
namespace google {
namespace protobuf {

template <>
inline const EnumDescriptor* GetEnumDescriptor< ::lunabotics::AllWheelControl_AllWheelControlType>() {
  return ::lunabotics::AllWheelControl_AllWheelControlType_descriptor();
}
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::lunabotics::AllWheelControl_PredefinedControlType>() {
  return ::lunabotics::AllWheelControl_PredefinedControlType_descriptor();
}

}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_AllWheelControl_2eproto__INCLUDED
