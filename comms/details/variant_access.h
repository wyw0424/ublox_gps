//
// Copyright 2017 - 2019 (C). Alex Robenko. All rights reserved.
//

// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.


#pragma once

#include <tuple>
#include <type_traits>

#include "macro_common.h"
#include "gen_enum.h"
#include "base_detection.h"

#ifdef COMMS_MUST_DEFINE_BASE
#define COMMS_AS_VARIANT_FUNC Base& asVariant()
#define COMMS_AS_VARIANT_CONST_FUNC const Base& asVariant() const
#define COMMS_VARIANT_INIT_FIELD_FUNC(v_, n_) \
    typename std::tuple_element<COMMS_CONCATENATE(FieldIdx_, n_), typename Base::Members>::type& COMMS_CONCATENATE(initField_, n_)(TArgs&&... args)
#define COMMS_VARIANT_ACCESS_FIELD_FUNC(v_, n_) \
    typename std::tuple_element<COMMS_CONCATENATE(FieldIdx_, n_), typename Base::Members>::type& COMMS_CONCATENATE(accessField_, n_)()
#define COMMS_VARIANT_ACCESS_FIELD_CONST_FUNC(v_, n_) \
    const typename std::tuple_element<COMMS_CONCATENATE(FieldIdx_, n_), typename Base::Members>::type& COMMS_CONCATENATE(accessField_, n_)() const

#else // #ifdef COMMS_MUST_DEFINE_BASE
#define COMMS_AS_VARIANT_FUNC FUNC_AUTO_REF_RETURN(asVariant, decltype(comms::field::toFieldBase(*this)))
#define COMMS_AS_VARIANT_CONST_FUNC FUNC_AUTO_REF_RETURN_CONST(asVariant, decltype(comms::field::toFieldBase(*this)))
#define COMMS_VARIANT_INIT_FIELD_FUNC(v_, n_) \
    FUNC_ARGS_AUTO_REF_RETURN(COMMS_CONCATENATE(initField_, n_), TArgs&&... args, decltype(v_.template initField<COMMS_CONCATENATE(FieldIdx_, n_)>(std::forward<TArgs>(args)...)))
#define COMMS_VARIANT_ACCESS_FIELD_FUNC(v_, n_) \
    FUNC_AUTO_REF_RETURN(COMMS_CONCATENATE(accessField_, n_), decltype(v_.template accessField<COMMS_CONCATENATE(FieldIdx_, n_)>()))
#define COMMS_VARIANT_ACCESS_FIELD_CONST_FUNC(v_, n_) \
    FUNC_AUTO_REF_RETURN_CONST(COMMS_CONCATENATE(accessField_, n_), decltype(v_.template accessField<COMMS_CONCATENATE(FieldIdx_, n_)>()))
#endif // #ifdef COMMS_MUST_DEFINE_BASE

#define COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    template <typename... TArgs> \
    COMMS_VARIANT_INIT_FIELD_FUNC(v_, n_) {\
        return v_.template initField<COMMS_CONCATENATE(FieldIdx_, n_)>(std::forward<TArgs>(args)...); \
    } \
    COMMS_VARIANT_ACCESS_FIELD_FUNC(v_, n_) { \
        return v_.template accessField<COMMS_CONCATENATE(FieldIdx_, n_)>(); \
    } \
    COMMS_VARIANT_ACCESS_FIELD_CONST_FUNC(v_, n_) { \
        return v_.template accessField<COMMS_CONCATENATE(FieldIdx_, n_)>(); \
    }

#define COMMS_VARIANT_MEM_ACC_FUNC_1(v_, n_) COMMS_VARIANT_MEM_ACC_FUNC(v_, n_)
#define COMMS_VARIANT_MEM_ACC_FUNC_2(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_1(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_3(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_2(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_4(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_3(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_5(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_4(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_6(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_5(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_7(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_6(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_8(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_7(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_9(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_8(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_10(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_9(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_11(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_10(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_12(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_11(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_13(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_12(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_14(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_13(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_15(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_14(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_16(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_15(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_17(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_16(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_18(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_17(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_19(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_18(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_20(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_19(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_21(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_20(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_22(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_21(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_23(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_22(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_24(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_23(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_25(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_24(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_26(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_25(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_27(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_26(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_28(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_27(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_29(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_28(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_30(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_29(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_31(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_30(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_32(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_31(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_33(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_32(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_34(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_33(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_35(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_34(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_36(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_35(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_37(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_36(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_38(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_37(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_39(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_38(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_40(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_39(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_41(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_40(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_42(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_41(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_43(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_42(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_44(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_43(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_45(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_44(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_46(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_45(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_47(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_46(v_, __VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_48(v_, n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC(v_, n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_47(v_, __VA_ARGS__))

#define COMMS_CHOOSE_VARIANT_MEM_ACC_FUNC_(N, v_, ...) COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_ ## N(v_, __VA_ARGS__))
#define COMMS_CHOOSE_VARIANT_MEM_ACC_FUNC(N, v_, ...) COMMS_EXPAND(COMMS_CHOOSE_VARIANT_MEM_ACC_FUNC_(N, v_, __VA_ARGS__))
#define COMMS_DO_VARIANT_MEM_ACC_FUNC(v_, ...) \
    COMMS_EXPAND(COMMS_CHOOSE_VARIANT_MEM_ACC_FUNC(COMMS_NUM_ARGS(__VA_ARGS__), v_, __VA_ARGS__))

// --------------------------------------------

#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    template <typename... TArgs> \
    typename std::tuple_element<COMMS_CONCATENATE(FieldIdx_, n_), Members>::type& COMMS_CONCATENATE(initField_, n_)(TArgs&&... args) {\
        return initField<COMMS_CONCATENATE(FieldIdx_, n_)>(std::forward<TArgs>(args)...); \
    } \
    typename std::tuple_element<COMMS_CONCATENATE(FieldIdx_, n_), Members>::type& COMMS_CONCATENATE(accessField_, n_)() { \
        return accessField<COMMS_CONCATENATE(FieldIdx_, n_)>(); \
    } \
    const typename std::tuple_element<COMMS_CONCATENATE(FieldIdx_, n_), Members>::type& COMMS_CONCATENATE(accessField_, n_)() const { \
        return accessField<COMMS_CONCATENATE(FieldIdx_, n_)>(); \
    }

#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_1(n_) COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_)
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_2(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_1(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_3(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_2(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_4(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_3(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_5(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_4(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_6(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_5(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_7(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_6(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_8(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_7(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_9(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_8(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_10(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_9(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_11(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_10(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_12(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_11(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_13(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_12(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_14(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_13(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_15(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_14(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_16(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_15(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_17(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_16(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_18(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_17(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_19(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_18(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_20(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_19(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_21(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_20(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_22(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_21(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_23(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_22(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_24(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_23(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_25(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_24(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_26(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_25(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_27(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_26(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_28(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_27(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_29(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_28(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_30(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_29(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_31(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_30(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_32(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_31(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_33(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_32(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_34(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_33(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_35(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_34(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_36(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_35(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_37(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_36(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_38(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_37(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_39(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_38(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_40(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_39(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_41(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_40(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_42(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_41(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_43(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_42(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_44(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_43(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_45(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_44(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_46(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_45(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_47(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_46(__VA_ARGS__))
#define COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_48(n_, ...) \
    COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(n_) \
    COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_47(__VA_ARGS__))

#define COMMS_CHOOSE_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_(N, ...) COMMS_EXPAND(COMMS_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_ ## N(__VA_ARGS__))
#define COMMS_CHOOSE_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(N, ...) COMMS_EXPAND(COMMS_CHOOSE_VARIANT_MEM_ACC_FUNC_NOTEMPLATE_(N, __VA_ARGS__))
#define COMMS_DO_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(...) \
    COMMS_EXPAND(COMMS_CHOOSE_VARIANT_MEM_ACC_FUNC_NOTEMPLATE(COMMS_NUM_ARGS(__VA_ARGS__), __VA_ARGS__))











