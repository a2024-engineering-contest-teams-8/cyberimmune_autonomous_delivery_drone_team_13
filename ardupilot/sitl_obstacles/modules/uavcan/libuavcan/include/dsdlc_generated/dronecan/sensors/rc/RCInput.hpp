/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/user/cyberimmune-systems-autonomous-delivery-drone-with-kos-contest/modules/DroneCAN/DSDL/dronecan/sensors/rc/1140.RCInput.uavcan
 */

#ifndef DRONECAN_SENSORS_RC_RCINPUT_HPP_INCLUDED
#define DRONECAN_SENSORS_RC_RCINPUT_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
uint8 STATUS_QUALITY_VALID = 1 # quality field is valid
uint8 STATUS_FAILSAFE = 2 # receiver has lost contact with transmitter

uint16 status      # bitmask of status bits, enumerated above with STATUS_*

uint8 quality      # scaled, 0 is no signal, 255 is "full" signal
uint4 id           # ID of this RC input device

uint12[<=32] rcin  # RC channel values between 0 and 4095
******************************************************************************/

/********************* DSDL signature source definition ***********************
dronecan.sensors.rc.RCInput
saturated uint16 status
saturated uint8 quality
saturated uint4 id
saturated uint12[<=32] rcin
******************************************************************************/

#undef status
#undef quality
#undef id
#undef rcin
#undef STATUS_QUALITY_VALID
#undef STATUS_FAILSAFE

namespace dronecan
{
namespace sensors
{
namespace rc
{

template <int _tmpl>
struct UAVCAN_EXPORT RCInput_
{
    typedef const RCInput_<_tmpl>& ParameterType;
    typedef RCInput_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_QUALITY_VALID;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FAILSAFE;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > status;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > quality;
        typedef ::uavcan::IntegerSpec< 4, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > id;
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 12, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 32 > rcin;
    };

    enum
    {
        MinBitLen
            = FieldTypes::status::MinBitLen
            + FieldTypes::quality::MinBitLen
            + FieldTypes::id::MinBitLen
            + FieldTypes::rcin::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::status::MaxBitLen
            + FieldTypes::quality::MaxBitLen
            + FieldTypes::id::MaxBitLen
            + FieldTypes::rcin::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_QUALITY_VALID >::Type STATUS_QUALITY_VALID; // 1
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FAILSAFE >::Type STATUS_FAILSAFE; // 2

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::status >::Type status;
    typename ::uavcan::StorageType< typename FieldTypes::quality >::Type quality;
    typename ::uavcan::StorageType< typename FieldTypes::id >::Type id;
    typename ::uavcan::StorageType< typename FieldTypes::rcin >::Type rcin;

    RCInput_()
        : status()
        , quality()
        , id()
        , rcin()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<418 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 1140 };

    static const char* getDataTypeFullName()
    {
        return "dronecan.sensors.rc.RCInput";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool RCInput_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        status == rhs.status &&
        quality == rhs.quality &&
        id == rhs.id &&
        rcin == rhs.rcin;
}

template <int _tmpl>
bool RCInput_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(status, rhs.status) &&
        ::uavcan::areClose(quality, rhs.quality) &&
        ::uavcan::areClose(id, rhs.id) &&
        ::uavcan::areClose(rcin, rhs.rcin);
}

template <int _tmpl>
int RCInput_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::status::encode(self.status, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::quality::encode(self.quality, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::id::encode(self.id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::rcin::encode(self.rcin, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int RCInput_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::status::decode(self.status, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::quality::decode(self.quality, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::id::decode(self.id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::rcin::decode(self.rcin, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature RCInput_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x771555E596AAB4CFULL);

    FieldTypes::status::extendDataTypeSignature(signature);
    FieldTypes::quality::extendDataTypeSignature(signature);
    FieldTypes::id::extendDataTypeSignature(signature);
    FieldTypes::rcin::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename RCInput_<_tmpl>::ConstantTypes::STATUS_QUALITY_VALID >::Type
    RCInput_<_tmpl>::STATUS_QUALITY_VALID = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename RCInput_<_tmpl>::ConstantTypes::STATUS_FAILSAFE >::Type
    RCInput_<_tmpl>::STATUS_FAILSAFE = 2U; // 2

/*
 * Final typedef
 */
typedef RCInput_<0> RCInput;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::dronecan::sensors::rc::RCInput > _uavcan_gdtr_registrator_RCInput;

}

} // Namespace rc
} // Namespace sensors
} // Namespace dronecan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::dronecan::sensors::rc::RCInput >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::dronecan::sensors::rc::RCInput::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::dronecan::sensors::rc::RCInput >::stream(Stream& s, ::dronecan::sensors::rc::RCInput::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "status: ";
    YamlStreamer< ::dronecan::sensors::rc::RCInput::FieldTypes::status >::stream(s, obj.status, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "quality: ";
    YamlStreamer< ::dronecan::sensors::rc::RCInput::FieldTypes::quality >::stream(s, obj.quality, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "id: ";
    YamlStreamer< ::dronecan::sensors::rc::RCInput::FieldTypes::id >::stream(s, obj.id, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "rcin: ";
    YamlStreamer< ::dronecan::sensors::rc::RCInput::FieldTypes::rcin >::stream(s, obj.rcin, level + 1);
}

}

namespace dronecan
{
namespace sensors
{
namespace rc
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::dronecan::sensors::rc::RCInput::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::dronecan::sensors::rc::RCInput >::stream(s, obj, 0);
    return s;
}

} // Namespace rc
} // Namespace sensors
} // Namespace dronecan

#endif // DRONECAN_SENSORS_RC_RCINPUT_HPP_INCLUDED