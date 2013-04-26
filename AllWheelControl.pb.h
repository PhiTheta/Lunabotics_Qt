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
#include "AllWheelState.pb.h"
#include "Point.pb.h"
// @@protoc_insertion_point(includes)

namespace lunabotics {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_AllWheelControl_2eproto();
void protobuf_AssignDesc_AllWheelControl_2eproto();
void protobuf_ShutdownFile_AllWheelControl_2eproto();

class AllWheelControl;
class AllWheelControl_PredefinedControl;
class AllWheelControl_ICRControl;

enum AllWheelControl_AllWheelControlType {
  AllWheelControl_AllWheelControlType_EXPLICIT = 1,
  AllWheelControl_AllWheelControlType_PREDEFINED = 2,
  AllWheelControl_AllWheelControlType_ICR = 3
};
bool AllWheelControl_AllWheelControlType_IsValid(int value);
const AllWheelControl_AllWheelControlType AllWheelControl_AllWheelControlType_AllWheelControlType_MIN = AllWheelControl_AllWheelControlType_EXPLICIT;
const AllWheelControl_AllWheelControlType AllWheelControl_AllWheelControlType_AllWheelControlType_MAX = AllWheelControl_AllWheelControlType_ICR;
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
  AllWheelControl_PredefinedControlType_DRIVE_FORWARD = 1,
  AllWheelControl_PredefinedControlType_DRIVE_BACKWARD = 2,
  AllWheelControl_PredefinedControlType_CRAB_LEFT = 3,
  AllWheelControl_PredefinedControlType_CRAB_RIGHT = 4,
  AllWheelControl_PredefinedControlType_TURN_CW = 5,
  AllWheelControl_PredefinedControlType_TURN_CCW = 6
};
bool AllWheelControl_PredefinedControlType_IsValid(int value);
const AllWheelControl_PredefinedControlType AllWheelControl_PredefinedControlType_PredefinedControlType_MIN = AllWheelControl_PredefinedControlType_DRIVE_FORWARD;
const AllWheelControl_PredefinedControlType AllWheelControl_PredefinedControlType_PredefinedControlType_MAX = AllWheelControl_PredefinedControlType_TURN_CCW;
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

class AllWheelControl_ICRControl : public ::google::protobuf::Message {
 public:
  AllWheelControl_ICRControl();
  virtual ~AllWheelControl_ICRControl();

  AllWheelControl_ICRControl(const AllWheelControl_ICRControl& from);

  inline AllWheelControl_ICRControl& operator=(const AllWheelControl_ICRControl& from) {
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
  static const AllWheelControl_ICRControl& default_instance();

  void Swap(AllWheelControl_ICRControl* other);

  // implements Message ----------------------------------------------

  AllWheelControl_ICRControl* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const AllWheelControl_ICRControl& from);
  void MergeFrom(const AllWheelControl_ICRControl& from);
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

  // required .lunabotics.Point icr = 1;
  inline bool has_icr() const;
  inline void clear_icr();
  static const int kIcrFieldNumber = 1;
  inline const ::lunabotics::Point& icr() const;
  inline ::lunabotics::Point* mutable_icr();
  inline ::lunabotics::Point* release_icr();
  inline void set_allocated_icr(::lunabotics::Point* icr);

  // required float velocity = 2;
  inline bool has_velocity() const;
  inline void clear_velocity();
  static const int kVelocityFieldNumber = 2;
  inline float velocity() const;
  inline void set_velocity(float value);

  // @@protoc_insertion_point(class_scope:lunabotics.AllWheelControl.ICRControl)
 private:
  inline void set_has_icr();
  inline void clear_has_icr();
  inline void set_has_velocity();
  inline void clear_has_velocity();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::lunabotics::Point* icr_;
  float velocity_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(2 + 31) / 32];

  friend void  protobuf_AddDesc_AllWheelControl_2eproto();
  friend void protobuf_AssignDesc_AllWheelControl_2eproto();
  friend void protobuf_ShutdownFile_AllWheelControl_2eproto();

  void InitAsDefaultInstance();
  static AllWheelControl_ICRControl* default_instance_;
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

  typedef AllWheelControl_PredefinedControl PredefinedControl;
  typedef AllWheelControl_ICRControl ICRControl;

  typedef AllWheelControl_AllWheelControlType AllWheelControlType;
  static const AllWheelControlType EXPLICIT = AllWheelControl_AllWheelControlType_EXPLICIT;
  static const AllWheelControlType PREDEFINED = AllWheelControl_AllWheelControlType_PREDEFINED;
  static const AllWheelControlType ICR = AllWheelControl_AllWheelControlType_ICR;
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
  static const PredefinedControlType DRIVE_FORWARD = AllWheelControl_PredefinedControlType_DRIVE_FORWARD;
  static const PredefinedControlType DRIVE_BACKWARD = AllWheelControl_PredefinedControlType_DRIVE_BACKWARD;
  static const PredefinedControlType CRAB_LEFT = AllWheelControl_PredefinedControlType_CRAB_LEFT;
  static const PredefinedControlType CRAB_RIGHT = AllWheelControl_PredefinedControlType_CRAB_RIGHT;
  static const PredefinedControlType TURN_CW = AllWheelControl_PredefinedControlType_TURN_CW;
  static const PredefinedControlType TURN_CCW = AllWheelControl_PredefinedControlType_TURN_CCW;
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

  // optional .lunabotics.AllWheelState explicit_data = 2;
  inline bool has_explicit_data() const;
  inline void clear_explicit_data();
  static const int kExplicitDataFieldNumber = 2;
  inline const ::lunabotics::AllWheelState& explicit_data() const;
  inline ::lunabotics::AllWheelState* mutable_explicit_data();
  inline ::lunabotics::AllWheelState* release_explicit_data();
  inline void set_allocated_explicit_data(::lunabotics::AllWheelState* explicit_data);

  // optional .lunabotics.AllWheelControl.PredefinedControl predefined_data = 3;
  inline bool has_predefined_data() const;
  inline void clear_predefined_data();
  static const int kPredefinedDataFieldNumber = 3;
  inline const ::lunabotics::AllWheelControl_PredefinedControl& predefined_data() const;
  inline ::lunabotics::AllWheelControl_PredefinedControl* mutable_predefined_data();
  inline ::lunabotics::AllWheelControl_PredefinedControl* release_predefined_data();
  inline void set_allocated_predefined_data(::lunabotics::AllWheelControl_PredefinedControl* predefined_data);

  // optional .lunabotics.AllWheelControl.ICRControl icr_data = 4;
  inline bool has_icr_data() const;
  inline void clear_icr_data();
  static const int kIcrDataFieldNumber = 4;
  inline const ::lunabotics::AllWheelControl_ICRControl& icr_data() const;
  inline ::lunabotics::AllWheelControl_ICRControl* mutable_icr_data();
  inline ::lunabotics::AllWheelControl_ICRControl* release_icr_data();
  inline void set_allocated_icr_data(::lunabotics::AllWheelControl_ICRControl* icr_data);

  // @@protoc_insertion_point(class_scope:lunabotics.AllWheelControl)
 private:
  inline void set_has_all_wheel_type();
  inline void clear_has_all_wheel_type();
  inline void set_has_explicit_data();
  inline void clear_has_explicit_data();
  inline void set_has_predefined_data();
  inline void clear_has_predefined_data();
  inline void set_has_icr_data();
  inline void clear_has_icr_data();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::lunabotics::AllWheelState* explicit_data_;
  ::lunabotics::AllWheelControl_PredefinedControl* predefined_data_;
  ::lunabotics::AllWheelControl_ICRControl* icr_data_;
  int all_wheel_type_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(4 + 31) / 32];

  friend void  protobuf_AddDesc_AllWheelControl_2eproto();
  friend void protobuf_AssignDesc_AllWheelControl_2eproto();
  friend void protobuf_ShutdownFile_AllWheelControl_2eproto();

  void InitAsDefaultInstance();
  static AllWheelControl* default_instance_;
};
// ===================================================================


// ===================================================================

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

// AllWheelControl_ICRControl

// required .lunabotics.Point icr = 1;
inline bool AllWheelControl_ICRControl::has_icr() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void AllWheelControl_ICRControl::set_has_icr() {
  _has_bits_[0] |= 0x00000001u;
}
inline void AllWheelControl_ICRControl::clear_has_icr() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void AllWheelControl_ICRControl::clear_icr() {
  if (icr_ != NULL) icr_->::lunabotics::Point::Clear();
  clear_has_icr();
}
inline const ::lunabotics::Point& AllWheelControl_ICRControl::icr() const {
  return icr_ != NULL ? *icr_ : *default_instance_->icr_;
}
inline ::lunabotics::Point* AllWheelControl_ICRControl::mutable_icr() {
  set_has_icr();
  if (icr_ == NULL) icr_ = new ::lunabotics::Point;
  return icr_;
}
inline ::lunabotics::Point* AllWheelControl_ICRControl::release_icr() {
  clear_has_icr();
  ::lunabotics::Point* temp = icr_;
  icr_ = NULL;
  return temp;
}
inline void AllWheelControl_ICRControl::set_allocated_icr(::lunabotics::Point* icr) {
  delete icr_;
  icr_ = icr;
  if (icr) {
    set_has_icr();
  } else {
    clear_has_icr();
  }
}

// required float velocity = 2;
inline bool AllWheelControl_ICRControl::has_velocity() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void AllWheelControl_ICRControl::set_has_velocity() {
  _has_bits_[0] |= 0x00000002u;
}
inline void AllWheelControl_ICRControl::clear_has_velocity() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void AllWheelControl_ICRControl::clear_velocity() {
  velocity_ = 0;
  clear_has_velocity();
}
inline float AllWheelControl_ICRControl::velocity() const {
  return velocity_;
}
inline void AllWheelControl_ICRControl::set_velocity(float value) {
  set_has_velocity();
  velocity_ = value;
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

// optional .lunabotics.AllWheelState explicit_data = 2;
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
  if (explicit_data_ != NULL) explicit_data_->::lunabotics::AllWheelState::Clear();
  clear_has_explicit_data();
}
inline const ::lunabotics::AllWheelState& AllWheelControl::explicit_data() const {
  return explicit_data_ != NULL ? *explicit_data_ : *default_instance_->explicit_data_;
}
inline ::lunabotics::AllWheelState* AllWheelControl::mutable_explicit_data() {
  set_has_explicit_data();
  if (explicit_data_ == NULL) explicit_data_ = new ::lunabotics::AllWheelState;
  return explicit_data_;
}
inline ::lunabotics::AllWheelState* AllWheelControl::release_explicit_data() {
  clear_has_explicit_data();
  ::lunabotics::AllWheelState* temp = explicit_data_;
  explicit_data_ = NULL;
  return temp;
}
inline void AllWheelControl::set_allocated_explicit_data(::lunabotics::AllWheelState* explicit_data) {
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

// optional .lunabotics.AllWheelControl.ICRControl icr_data = 4;
inline bool AllWheelControl::has_icr_data() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void AllWheelControl::set_has_icr_data() {
  _has_bits_[0] |= 0x00000008u;
}
inline void AllWheelControl::clear_has_icr_data() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void AllWheelControl::clear_icr_data() {
  if (icr_data_ != NULL) icr_data_->::lunabotics::AllWheelControl_ICRControl::Clear();
  clear_has_icr_data();
}
inline const ::lunabotics::AllWheelControl_ICRControl& AllWheelControl::icr_data() const {
  return icr_data_ != NULL ? *icr_data_ : *default_instance_->icr_data_;
}
inline ::lunabotics::AllWheelControl_ICRControl* AllWheelControl::mutable_icr_data() {
  set_has_icr_data();
  if (icr_data_ == NULL) icr_data_ = new ::lunabotics::AllWheelControl_ICRControl;
  return icr_data_;
}
inline ::lunabotics::AllWheelControl_ICRControl* AllWheelControl::release_icr_data() {
  clear_has_icr_data();
  ::lunabotics::AllWheelControl_ICRControl* temp = icr_data_;
  icr_data_ = NULL;
  return temp;
}
inline void AllWheelControl::set_allocated_icr_data(::lunabotics::AllWheelControl_ICRControl* icr_data) {
  delete icr_data_;
  icr_data_ = icr_data;
  if (icr_data) {
    set_has_icr_data();
  } else {
    clear_has_icr_data();
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
