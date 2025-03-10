// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

// TypedReference is basically only ever seen on the call stack, and in param arrays.
// These are blob that must be dealt with by the compiler.

using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.Versioning;

namespace System
{
    [NonVersionable] // This only applies to field layout
    public ref partial struct TypedReference
    {
        private readonly ref byte _value;
        private readonly IntPtr _type;

        // implementation of CORINFO_HELP_GETREFANY
        [StackTraceHidden]
        internal static ref byte GetRefAny(IntPtr clsHnd, TypedReference typedByRef)
        {
            if (clsHnd != typedByRef._type)
            {
                ThrowInvalidCastException();
            }

            return ref typedByRef._value;

            [DoesNotReturn]
            [StackTraceHidden]
            static void ThrowInvalidCastException() => throw new InvalidCastException();
        }

        private TypedReference(ref byte target, RuntimeType type)
        {
            _value = ref target;
            _type = type.GetUnderlyingNativeHandle();
        }

        public static unsafe object? ToObject(TypedReference value)
        {
            TypeHandle typeHandle = new((void*)value._type);

            if (typeHandle.IsNull)
            {
                ThrowHelper.ThrowArgumentException_ArgumentNull_TypedRefType();
            }

            // The only case where a type handle here might be a type desc is when the type is either a
            // pointer or a function pointer. In those cases, just always return the method table pointer
            // for System.UIntPtr without inspecting the type desc any further. Otherwise, the type handle
            // is just wrapping a method table pointer, so return that directly with a reinterpret cast.
            MethodTable* pMethodTable = typeHandle.IsTypeDesc
                ? TypeHandle.TypeHandleOf<UIntPtr>().AsMethodTable()
                : typeHandle.AsMethodTable();

            Debug.Assert(pMethodTable is not null);

            object? result;

            if (pMethodTable->IsValueType)
            {
                result = RuntimeHelpers.Box(pMethodTable, ref value._value);
            }
            else
            {
                result = Unsafe.As<byte, object>(ref value._value);
            }

            return result;
        }
    }
}
