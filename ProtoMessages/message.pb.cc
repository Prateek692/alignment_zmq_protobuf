// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: message.proto

#include "message.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
extern PROTOBUF_INTERNAL_EXPORT_message_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Coordinates_Coordinate_message_2eproto;
class Coordinates_CoordinateDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Coordinates_Coordinate> _instance;
} _Coordinates_Coordinate_default_instance_;
class CoordinatesDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Coordinates> _instance;
} _Coordinates_default_instance_;
static void InitDefaultsscc_info_Coordinates_message_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::_Coordinates_default_instance_;
    new (ptr) ::Coordinates();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::Coordinates::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_Coordinates_message_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_Coordinates_message_2eproto}, {
      &scc_info_Coordinates_Coordinate_message_2eproto.base,}};

static void InitDefaultsscc_info_Coordinates_Coordinate_message_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::_Coordinates_Coordinate_default_instance_;
    new (ptr) ::Coordinates_Coordinate();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::Coordinates_Coordinate::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Coordinates_Coordinate_message_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_Coordinates_Coordinate_message_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_message_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_message_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_message_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_message_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::Coordinates_Coordinate, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::Coordinates_Coordinate, values_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::Coordinates, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::Coordinates, unit_),
  PROTOBUF_FIELD_OFFSET(::Coordinates, laser_type_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::Coordinates_Coordinate)},
  { 6, -1, sizeof(::Coordinates)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::_Coordinates_Coordinate_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::_Coordinates_default_instance_),
};

const char descriptor_table_protodef_message_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\rmessage.proto\"f\n\013Coordinates\022%\n\004unit\030\001"
  " \003(\0132\027.Coordinates.Coordinate\022\022\n\nLaser_t"
  "ype\030\002 \001(\t\032\034\n\nCoordinate\022\016\n\006values\030\001 \003(\001b"
  "\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_message_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_message_2eproto_sccs[2] = {
  &scc_info_Coordinates_message_2eproto.base,
  &scc_info_Coordinates_Coordinate_message_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_message_2eproto_once;
static bool descriptor_table_message_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_message_2eproto = {
  &descriptor_table_message_2eproto_initialized, descriptor_table_protodef_message_2eproto, "message.proto", 127,
  &descriptor_table_message_2eproto_once, descriptor_table_message_2eproto_sccs, descriptor_table_message_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_message_2eproto::offsets,
  file_level_metadata_message_2eproto, 2, file_level_enum_descriptors_message_2eproto, file_level_service_descriptors_message_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_message_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_message_2eproto), true);

// ===================================================================

void Coordinates_Coordinate::InitAsDefaultInstance() {
}
class Coordinates_Coordinate::_Internal {
 public:
};

Coordinates_Coordinate::Coordinates_Coordinate()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:Coordinates.Coordinate)
}
Coordinates_Coordinate::Coordinates_Coordinate(const Coordinates_Coordinate& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      values_(from.values_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:Coordinates.Coordinate)
}

void Coordinates_Coordinate::SharedCtor() {
}

Coordinates_Coordinate::~Coordinates_Coordinate() {
  // @@protoc_insertion_point(destructor:Coordinates.Coordinate)
  SharedDtor();
}

void Coordinates_Coordinate::SharedDtor() {
}

void Coordinates_Coordinate::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Coordinates_Coordinate& Coordinates_Coordinate::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Coordinates_Coordinate_message_2eproto.base);
  return *internal_default_instance();
}


void Coordinates_Coordinate::Clear() {
// @@protoc_insertion_point(message_clear_start:Coordinates.Coordinate)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  values_.Clear();
  _internal_metadata_.Clear();
}

const char* Coordinates_Coordinate::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated double values = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_values(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9) {
          _internal_add_values(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Coordinates_Coordinate::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:Coordinates.Coordinate)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated double values = 1;
  if (this->_internal_values_size() > 0) {
    target = stream->WriteFixedPacked(1, _internal_values(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:Coordinates.Coordinate)
  return target;
}

size_t Coordinates_Coordinate::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:Coordinates.Coordinate)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated double values = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_values_size());
    size_t data_size = 8UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
            static_cast<::PROTOBUF_NAMESPACE_ID::int32>(data_size));
    }
    int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(data_size);
    _values_cached_byte_size_.store(cached_size,
                                    std::memory_order_relaxed);
    total_size += data_size;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Coordinates_Coordinate::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:Coordinates.Coordinate)
  GOOGLE_DCHECK_NE(&from, this);
  const Coordinates_Coordinate* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Coordinates_Coordinate>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:Coordinates.Coordinate)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:Coordinates.Coordinate)
    MergeFrom(*source);
  }
}

void Coordinates_Coordinate::MergeFrom(const Coordinates_Coordinate& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:Coordinates.Coordinate)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  values_.MergeFrom(from.values_);
}

void Coordinates_Coordinate::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:Coordinates.Coordinate)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Coordinates_Coordinate::CopyFrom(const Coordinates_Coordinate& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:Coordinates.Coordinate)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Coordinates_Coordinate::IsInitialized() const {
  return true;
}

void Coordinates_Coordinate::InternalSwap(Coordinates_Coordinate* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  values_.InternalSwap(&other->values_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Coordinates_Coordinate::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void Coordinates::InitAsDefaultInstance() {
}
class Coordinates::_Internal {
 public:
};

Coordinates::Coordinates()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:Coordinates)
}
Coordinates::Coordinates(const Coordinates& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      unit_(from.unit_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  laser_type_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (!from._internal_laser_type().empty()) {
    laser_type_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.laser_type_);
  }
  // @@protoc_insertion_point(copy_constructor:Coordinates)
}

void Coordinates::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_Coordinates_message_2eproto.base);
  laser_type_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

Coordinates::~Coordinates() {
  // @@protoc_insertion_point(destructor:Coordinates)
  SharedDtor();
}

void Coordinates::SharedDtor() {
  laser_type_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void Coordinates::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Coordinates& Coordinates::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Coordinates_message_2eproto.base);
  return *internal_default_instance();
}


void Coordinates::Clear() {
// @@protoc_insertion_point(message_clear_start:Coordinates)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  unit_.Clear();
  laser_type_.ClearToEmptyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  _internal_metadata_.Clear();
}

const char* Coordinates::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .Coordinates.Coordinate unit = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_unit(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<10>(ptr));
        } else goto handle_unusual;
        continue;
      // string Laser_type = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_laser_type();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "Coordinates.Laser_type"));
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Coordinates::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:Coordinates)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .Coordinates.Coordinate unit = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_unit_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, this->_internal_unit(i), target, stream);
  }

  // string Laser_type = 2;
  if (this->laser_type().size() > 0) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_laser_type().data(), static_cast<int>(this->_internal_laser_type().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "Coordinates.Laser_type");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_laser_type(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:Coordinates)
  return target;
}

size_t Coordinates::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:Coordinates)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .Coordinates.Coordinate unit = 1;
  total_size += 1UL * this->_internal_unit_size();
  for (const auto& msg : this->unit_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // string Laser_type = 2;
  if (this->laser_type().size() > 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_laser_type());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Coordinates::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:Coordinates)
  GOOGLE_DCHECK_NE(&from, this);
  const Coordinates* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Coordinates>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:Coordinates)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:Coordinates)
    MergeFrom(*source);
  }
}

void Coordinates::MergeFrom(const Coordinates& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:Coordinates)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  unit_.MergeFrom(from.unit_);
  if (from.laser_type().size() > 0) {

    laser_type_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.laser_type_);
  }
}

void Coordinates::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:Coordinates)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Coordinates::CopyFrom(const Coordinates& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:Coordinates)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Coordinates::IsInitialized() const {
  return true;
}

void Coordinates::InternalSwap(Coordinates* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  unit_.InternalSwap(&other->unit_);
  laser_type_.Swap(&other->laser_type_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
}

::PROTOBUF_NAMESPACE_ID::Metadata Coordinates::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::Coordinates_Coordinate* Arena::CreateMaybeMessage< ::Coordinates_Coordinate >(Arena* arena) {
  return Arena::CreateInternal< ::Coordinates_Coordinate >(arena);
}
template<> PROTOBUF_NOINLINE ::Coordinates* Arena::CreateMaybeMessage< ::Coordinates >(Arena* arena) {
  return Arena::CreateInternal< ::Coordinates >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
