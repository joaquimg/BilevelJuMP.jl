# Function in this file are heavily inspired in IntervalArithmetic.jl,
# which is licensed under the MIT "Expat" License:
#
# Copyright (c) 2014-2021: David P. Sanders & Luis Benet
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import Base.:+, Base.:-, Base.:*

struct Interval{T}
    lo::T
    hi::T
end
function Interval(lo::T, hi::T) where {T<:Real}
    # if hi < lo <= hi + eps(T)
    #     lo = hi
    # end
    @assert lo <= hi
    return Interval{T}(lo, hi)
end

function Base.iszero(a::Interval)
    return iszero(a.hi) && iszero(a.lo)
end

+(a::Interval) = a
-(a::Interval) = Interval(-a.hi, -a.lo)

function +(a::Interval{T}, b::T) where {T<:Real}
    return Interval(a.lo + b, a.hi + b)
end
+(b::T, a::Interval{T}) where {T<:Real} = a + b

function -(a::Interval{T}, b::T) where {T<:Real}
    return Interval(a.lo - b, a.hi - b)
end
function -(b::T, a::Interval{T}) where {T<:Real}
    return Interval(b - a.hi, b - a.lo)
end

function +(a::Interval{T}, b::Interval{T}) where {T<:Real}
    return Interval(a.lo + b.lo, a.hi + b.hi)
end

function -(a::Interval{T}, b::Interval{T}) where {T<:Real}
    return Interval(a.lo - b.hi, a.hi - b.lo)
end

## Multiplication
function *(x::T, a::Interval{T}) where {T<:Real}
    (iszero(a) || iszero(x)) && return Interval(zero(T), zero(T))
    if x ≥ 0.0
        return Interval(a.lo * x, a.hi * x)
    else
        return Interval(a.hi * x, a.lo * x)
    end
end

*(a::Interval{T}, x::T) where {T<:Real} = x * a
