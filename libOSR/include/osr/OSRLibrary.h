#pragma once

#ifndef OSR_EXPORT
#  ifdef _WIN32
#    ifdef OSR_EXPORTS
       /* We are building this library */
#      define OSR_EXPORT __declspec(dllexport)
#    else
       /* We are using this library */
#      define OSR_EXPORT __declspec(dllimport)
#    endif
#  else
#    define OSR_EXPORT
#  endif
#endif