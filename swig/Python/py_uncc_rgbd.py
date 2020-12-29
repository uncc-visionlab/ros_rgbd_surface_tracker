# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_py_uncc_rgbd')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_py_uncc_rgbd')
    _py_uncc_rgbd = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_py_uncc_rgbd', [dirname(__file__)])
        except ImportError:
            import _py_uncc_rgbd
            return _py_uncc_rgbd
        try:
            _mod = imp.load_module('_py_uncc_rgbd', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _py_uncc_rgbd = swig_import_helper()
    del swig_import_helper
else:
    import _py_uncc_rgbd
del _swig_python_version_info

try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.

try:
    import builtins as __builtin__
except ImportError:
    import __builtin__

def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static):
        if _newclass:
            object.__setattr__(self, name, value)
        else:
            self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr(self, class_type, name):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    raise AttributeError("'%s' object has no attribute '%s'" % (class_type.__name__, name))


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except __builtin__.Exception:
    class _object:
        pass
    _newclass = 0


def new_intp():
    return _py_uncc_rgbd.new_intp()
new_intp = _py_uncc_rgbd.new_intp

def copy_intp(value):
    return _py_uncc_rgbd.copy_intp(value)
copy_intp = _py_uncc_rgbd.copy_intp

def delete_intp(obj):
    return _py_uncc_rgbd.delete_intp(obj)
delete_intp = _py_uncc_rgbd.delete_intp

def intp_assign(obj, value):
    return _py_uncc_rgbd.intp_assign(obj, value)
intp_assign = _py_uncc_rgbd.intp_assign

def intp_value(obj):
    return _py_uncc_rgbd.intp_value(obj)
intp_value = _py_uncc_rgbd.intp_value

def new_readPulseFuncp():
    return _py_uncc_rgbd.new_readPulseFuncp()
new_readPulseFuncp = _py_uncc_rgbd.new_readPulseFuncp

def copy_readPulseFuncp(value):
    return _py_uncc_rgbd.copy_readPulseFuncp(value)
copy_readPulseFuncp = _py_uncc_rgbd.copy_readPulseFuncp

def delete_readPulseFuncp(obj):
    return _py_uncc_rgbd.delete_readPulseFuncp(obj)
delete_readPulseFuncp = _py_uncc_rgbd.delete_readPulseFuncp

def readPulseFuncp_assign(obj, value):
    return _py_uncc_rgbd.readPulseFuncp_assign(obj, value)
readPulseFuncp_assign = _py_uncc_rgbd.readPulseFuncp_assign

def readPulseFuncp_value(obj):
    return _py_uncc_rgbd.readPulseFuncp_value(obj)
readPulseFuncp_value = _py_uncc_rgbd.readPulseFuncp_value

def new_meta_parametersp():
    return _py_uncc_rgbd.new_meta_parametersp()
new_meta_parametersp = _py_uncc_rgbd.new_meta_parametersp

def copy_meta_parametersp(value):
    return _py_uncc_rgbd.copy_meta_parametersp(value)
copy_meta_parametersp = _py_uncc_rgbd.copy_meta_parametersp

def delete_meta_parametersp(obj):
    return _py_uncc_rgbd.delete_meta_parametersp(obj)
delete_meta_parametersp = _py_uncc_rgbd.delete_meta_parametersp

def meta_parametersp_assign(obj, value):
    return _py_uncc_rgbd.meta_parametersp_assign(obj, value)
meta_parametersp_assign = _py_uncc_rgbd.meta_parametersp_assign

def meta_parametersp_value(obj):
    return _py_uncc_rgbd.meta_parametersp_value(obj)
meta_parametersp_value = _py_uncc_rgbd.meta_parametersp_value

def new_doublep():
    return _py_uncc_rgbd.new_doublep()
new_doublep = _py_uncc_rgbd.new_doublep

def copy_doublep(value):
    return _py_uncc_rgbd.copy_doublep(value)
copy_doublep = _py_uncc_rgbd.copy_doublep

def delete_doublep(obj):
    return _py_uncc_rgbd.delete_doublep(obj)
delete_doublep = _py_uncc_rgbd.delete_doublep

def doublep_assign(obj, value):
    return _py_uncc_rgbd.doublep_assign(obj, value)
doublep_assign = _py_uncc_rgbd.doublep_assign

def doublep_value(obj):
    return _py_uncc_rgbd.doublep_value(obj)
doublep_value = _py_uncc_rgbd.doublep_value

def new_int_array1d(nelements):
    return _py_uncc_rgbd.new_int_array1d(nelements)
new_int_array1d = _py_uncc_rgbd.new_int_array1d

def delete_int_array1d(ary):
    return _py_uncc_rgbd.delete_int_array1d(ary)
delete_int_array1d = _py_uncc_rgbd.delete_int_array1d

def int_array1d_getitem(ary, index):
    return _py_uncc_rgbd.int_array1d_getitem(ary, index)
int_array1d_getitem = _py_uncc_rgbd.int_array1d_getitem

def int_array1d_setitem(ary, index, value):
    return _py_uncc_rgbd.int_array1d_setitem(ary, index, value)
int_array1d_setitem = _py_uncc_rgbd.int_array1d_setitem

def new_double_array1d(nelements):
    return _py_uncc_rgbd.new_double_array1d(nelements)
new_double_array1d = _py_uncc_rgbd.new_double_array1d

def delete_double_array1d(ary):
    return _py_uncc_rgbd.delete_double_array1d(ary)
delete_double_array1d = _py_uncc_rgbd.delete_double_array1d

def double_array1d_getitem(ary, index):
    return _py_uncc_rgbd.double_array1d_getitem(ary, index)
double_array1d_getitem = _py_uncc_rgbd.double_array1d_getitem

def double_array1d_setitem(ary, index, value):
    return _py_uncc_rgbd.double_array1d_setitem(ary, index, value)
double_array1d_setitem = _py_uncc_rgbd.double_array1d_setitem

def new_int_array2d(nelements):
    return _py_uncc_rgbd.new_int_array2d(nelements)
new_int_array2d = _py_uncc_rgbd.new_int_array2d

def delete_int_array2d(ary):
    return _py_uncc_rgbd.delete_int_array2d(ary)
delete_int_array2d = _py_uncc_rgbd.delete_int_array2d

def int_array2d_getitem(ary, index):
    return _py_uncc_rgbd.int_array2d_getitem(ary, index)
int_array2d_getitem = _py_uncc_rgbd.int_array2d_getitem

def int_array2d_setitem(ary, index, value):
    return _py_uncc_rgbd.int_array2d_setitem(ary, index, value)
int_array2d_setitem = _py_uncc_rgbd.int_array2d_setitem

def new_double_array2d(nelements):
    return _py_uncc_rgbd.new_double_array2d(nelements)
new_double_array2d = _py_uncc_rgbd.new_double_array2d

def delete_double_array2d(ary):
    return _py_uncc_rgbd.delete_double_array2d(ary)
delete_double_array2d = _py_uncc_rgbd.delete_double_array2d

def double_array2d_getitem(ary, index):
    return _py_uncc_rgbd.double_array2d_getitem(ary, index)
double_array2d_getitem = _py_uncc_rgbd.double_array2d_getitem

def double_array2d_setitem(ary, index, value):
    return _py_uncc_rgbd.double_array2d_setitem(ary, index, value)
double_array2d_setitem = _py_uncc_rgbd.double_array2d_setitem
# This file is compatible with both classic and new-style classes.


