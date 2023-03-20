// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.X86;

namespace System.Text
{
    public partial class UTF8Encoding
    {
        /// <summary>
        /// A special instance of <see cref="UTF8Encoding"/> that is initialized with "don't throw on invalid sequences;
        /// perform <see cref="Rune.ReplacementChar"/> substitution instead" semantics. This type allows for devirtualization
        /// of calls made directly off of <see cref="Encoding.UTF8"/>. See https://github.com/dotnet/coreclr/pull/9230.
        /// </summary>
        internal sealed class UTF8EncodingSealed : UTF8Encoding
        {
            /// <summary>
            /// Maximum number of input elements we'll allow for going through the fast one-pass stackalloc code paths.
            /// </summary>
            private const int MaxSmallInputElementCount = 32;

            // Break even slightly above 256 for x64
            private static readonly uint GetBytesSmallInputMaxElementCount = Vector128.IsHardwareAccelerated ? 128u : 16u;

            public UTF8EncodingSealed(bool encoderShouldEmitUTF8Identifier) : base(encoderShouldEmitUTF8Identifier) { }

            public override ReadOnlySpan<byte> Preamble => _emitUTF8Identifier ? PreambleSpan : default;

            public override object Clone()
            {
                // The base implementation of Encoding.Clone calls object.MemberwiseClone and marks the new object mutable.
                // We don't want to do this because it violates the invariants we have set for the sealed type.
                // Instead, we'll create a new instance of the base UTF8Encoding type and mark it mutable.

                return new UTF8Encoding(_emitUTF8Identifier)
                {
                    IsReadOnly = false
                };
            }

            public override byte[] GetBytes(string s)
            {
                // This method is short and can be inlined, meaning that the null check below
                // might be elided if the JIT can prove not-null at the call site.

                if (s?.Length <= MaxSmallInputElementCount)
                {
                    return GetBytesForSmallInput(s);
                }
                else
                {
                    return base.GetBytes(s!); // make the base method responsible for the null check
                }
            }

            private unsafe byte[] GetBytesForSmallInput(string s)
            {
                Debug.Assert(s != null);
                Debug.Assert(s.Length <= MaxSmallInputElementCount);

                byte* pDestination = stackalloc byte[MaxSmallInputElementCount * MaxUtf8BytesPerChar];

                int sourceLength = s.Length; // hoist this to avoid having the JIT auto-insert null checks
                int bytesWritten;

                fixed (char* pSource = s)
                {
                    bytesWritten = GetBytesCommon(pSource, sourceLength, pDestination, MaxSmallInputElementCount * MaxUtf8BytesPerChar);
                    Debug.Assert(0 <= bytesWritten && bytesWritten <= s.Length * MaxUtf8BytesPerChar);
                }

                return new Span<byte>(ref *pDestination, bytesWritten).ToArray(); // this overload of Span ctor doesn't validate length
            }

            public override int GetMaxByteCount(int charCount)
            {
                // This is a specialization of UTF8Encoding.GetMaxByteCount
                // with the assumption that the default replacement fallback
                // emits 3 fallback bytes ([ EF BF BD ] = '\uFFFD') per
                // malformed input char in the worst case.

                if ((uint)charCount > (int.MaxValue / MaxUtf8BytesPerChar) - 1)
                {
                    // Move the throw out of the hot path to allow for inlining.
                    ThrowArgumentException(charCount);
                    static void ThrowArgumentException(int charCount)
                    {
                        throw new ArgumentOutOfRangeException(
                            paramName: nameof(charCount),
                            message: (charCount < 0) ? SR.ArgumentOutOfRange_NeedNonNegNum : SR.ArgumentOutOfRange_GetByteCountOverflow);
                    }
                }

                return (charCount * MaxUtf8BytesPerChar) + MaxUtf8BytesPerChar;
            }

            public override int GetMaxCharCount(int byteCount)
            {
                // This is a specialization of UTF8Encoding.GetMaxCharCount
                // with the assumption that the default replacement fallback
                // emits one fallback char ('\uFFFD') per malformed input
                // byte in the worst case.

                if ((uint)byteCount > int.MaxValue - 1)
                {
                    // Move the throw out of the hot path to allow for inlining.
                    ThrowArgumentException(byteCount);
                    static void ThrowArgumentException(int byteCount)
                    {
                        throw new ArgumentOutOfRangeException(
                            paramName: nameof(byteCount),
                            message: (byteCount < 0) ? SR.ArgumentOutOfRange_NeedNonNegNum : SR.ArgumentOutOfRange_GetCharCountOverflow);
                    }
                }

                return byteCount + 1;
            }

            public override string GetString(byte[] bytes)
            {
                // This method is short and can be inlined, meaning that the null check below
                // might be elided if the JIT can prove not-null at the call site.

                if (bytes?.Length <= MaxSmallInputElementCount)
                {
                    return GetStringForSmallInput(bytes);
                }
                else
                {
                    return base.GetString(bytes!); // make the base method responsible for the null check
                }
            }

            private unsafe string GetStringForSmallInput(byte[] bytes)
            {
                Debug.Assert(bytes != null);
                Debug.Assert(bytes.Length <= MaxSmallInputElementCount);

                char* pDestination = stackalloc char[MaxSmallInputElementCount]; // each byte produces at most one char

                int sourceLength = bytes.Length; // hoist this to avoid having the JIT auto-insert null checks
                int charsWritten;

                fixed (byte* pSource = bytes)
                {
                    charsWritten = GetCharsCommon(pSource, sourceLength, pDestination, MaxSmallInputElementCount);
                    Debug.Assert(0 <= charsWritten && charsWritten <= sourceLength); // should never have more output chars than input bytes
                }

                return new string(new ReadOnlySpan<char>(ref *pDestination, charsWritten)); // this overload of ROS ctor doesn't validate length
            }

            public override unsafe int GetBytes(char* chars, int charCount, byte* bytes, int byteCount)
            {
                if (chars != null
                    && bytes != null
                    && byteCount >= charCount
                    // Break even slightly above 256 for x64
                    && (uint)charCount <= GetBytesSmallInputMaxElementCount)
                {
                    return GetBytesForSmallInput(chars, charCount, bytes, byteCount);
                }
                else
                {
                    return base.GetBytes(chars, charCount, bytes, byteCount);
                }
            }

            public override unsafe int GetBytes(ReadOnlySpan<char> chars, Span<byte> bytes)
            {
                fixed (char* charsPtr = &MemoryMarshal.GetReference(chars))
                fixed (byte* bytesPtr = &MemoryMarshal.GetReference(bytes))
                {
                    if (chars.Length <= bytes.Length
                        && (uint)chars.Length <= GetBytesSmallInputMaxElementCount)
                    {
                        return GetBytesForSmallInput(charsPtr, chars.Length, bytesPtr, bytes.Length);
                    }
                    else
                    {
                        return GetBytesCommon(charsPtr, chars.Length, bytesPtr, bytes.Length);
                    }
                }
            }

            // Read mask from static variable to prevent multiple reads when using variable
            private static readonly Vector128<ushort> VectorContainsNonAsciiCharMask = Vector128.Create(unchecked((ushort)0xff80));

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private unsafe int GetBytesForSmallInput(char* chars, int charCount, byte* bytes, int byteCount)
            {
                uint i = 0;
                if (Vector128.IsHardwareAccelerated && charCount >= Vector128<ushort>.Count)
                {
                    // Read mask from static variable to prevent multiple reads when using variable
                    Vector128<ushort> mask = VectorContainsNonAsciiCharMask;

                    uint maxSimdIndex = (uint)(charCount - Vector128<ushort>.Count);
                    for (i = 0; i < maxSimdIndex; i += (uint)Vector128<ushort>.Count)
                    {
                        Vector128<ushort> v = *(Vector128<ushort>*)(chars + i);
                        if (VectorContainsNonAsciiChar(v, mask))
                            goto NonAscii;

                        StoreLower((long*)(bytes + i), ExtractAsciiVector(v, v));
                    }

                    // Read last full vector and do a (possibly overlapping) store if successfull
                    Vector128<ushort> v2 = *(Vector128<ushort>*)(chars + maxSimdIndex);
                    if (VectorContainsNonAsciiChar(v2, mask))
                        goto NonAscii;

                    Vector128<byte> packed = ExtractAsciiVector(v2, v2);
                    StoreLower((long*)(bytes + charCount - sizeof(long)), packed);
                    return charCount;
                }
                else
                {
                    for (; i < (uint)charCount; ++i)
                    {
                        char t = chars[i];
                        if (t >= 0x80)
                            goto NonAscii;

                        bytes[i] = (byte)t;
                    }
                    return charCount;
                }

            NonAscii:
                return (int)i + GetBytesCommonNoInline(chars + i, charCount - (int)i, bytes + i, byteCount - (int)i);
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            private unsafe int GetBytesCommonNoInline(char* pChars, int charCount, byte* pBytes, int byteCount)
                => GetBytesCommon(pChars, charCount, pBytes, byteCount);

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static unsafe void StoreLower(long* address, Vector128<byte> source)
            {
                // Allow a single 8 byte store on 32bit for x86
                if (Sse2.IsSupported)
                    Sse2.StoreScalar(address, source.AsInt64());
                else
                    *address = source.AsInt64().ToScalar();
            }

            // Is it OK to make this System.Text.Ascii.ExtractAsciiVector internal and use it instead ?
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static Vector128<byte> ExtractAsciiVector(Vector128<ushort> vectorFirst, Vector128<ushort> vectorSecond)
            {
                // Narrows two vectors of words [ w7 w6 w5 w4 w3 w2 w1 w0 ] and [ w7' w6' w5' w4' w3' w2' w1' w0' ]
                // to a vector of bytes [ b7 ... b0 b7' ... b0'].

                // prefer architecture specific intrinsic as they don't perform additional AND like Vector128.Narrow does
                if (Sse2.IsSupported)
                {
                    return Sse2.PackUnsignedSaturate(vectorFirst.AsInt16(), vectorSecond.AsInt16());
                }
                else if (AdvSimd.Arm64.IsSupported)
                {
                    return AdvSimd.Arm64.UnzipEven(vectorFirst.AsByte(), vectorSecond.AsByte());
                }
                else
                {
                    return Vector128.Narrow(vectorFirst, vectorSecond);
                }
            }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static bool VectorContainsNonAsciiChar(Vector128<ushort> utf16Vector, Vector128<ushort> mask)
            {
                // prefer architecture specific intrinsic as they offer better perf
                if (Sse41.IsSupported)
                {
                    // If a non-ASCII bit is set in any WORD of the vector, we have seen non-ASCII data.
                    return !Sse41.TestZ(utf16Vector, mask);
                }
                else if (AdvSimd.Arm64.IsSupported)
                {
                    // First we pick four chars, a larger one from all four pairs of adjecent chars in the vector.
                    // If any of those four chars has a non-ASCII bit set, we have seen non-ASCII data.
                    Vector128<ushort> maxChars = AdvSimd.Arm64.MaxPairwise(utf16Vector, utf16Vector);
                    return (maxChars.AsUInt64().ToScalar() & 0xFF80FF80FF80FF80) != 0;
                }
                else
                {
                    // If a non-ASCII bit is set in any WORD of the vector, we have seen non-ASCII data.
                    return (utf16Vector & mask) != Vector128<ushort>.Zero;
                }
            }
        }
    }
}
