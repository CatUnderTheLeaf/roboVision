// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef im_publisher__VISIBILITY_H_
#define im_publisher__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define im_publisher_EXPORT __attribute__ ((dllexport))
    #define im_publisher_IMPORT __attribute__ ((dllimport))
  #else
    #define im_publisher_EXPORT __declspec(dllexport)
    #define im_publisher_IMPORT __declspec(dllimport)
  #endif

  #ifdef im_publisher_DLL
    #define im_publisher_PUBLIC im_publisher_EXPORT
  #else
    #define im_publisher_PUBLIC im_publisher_IMPORT
  #endif

  #define im_publisher_PUBLIC_TYPE im_publisher_PUBLIC

  #define im_publisher_LOCAL

#else

  #define im_publisher_EXPORT __attribute__ ((visibility("default")))
  #define im_publisher_IMPORT

  #if __GNUC__ >= 4
    #define im_publisher_PUBLIC __attribute__ ((visibility("default")))
    #define im_publisher_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define im_publisher_PUBLIC
    #define im_publisher_LOCAL
  #endif

  #define im_publisher_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // im_publisher__VISIBILITY_H_
