# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

from waflib.Configure import conf

FRIEND_TYPENAME = '''
class A;

template<typename T>
class B
{
  friend T;
};

B<A> g_b;
'''

FRIEND_WRAPPER = '''
class A;

template<typename T>
struct TypeWrapper
{
  typedef T Type;
};

template<typename T>
class B
{
  friend class TypeWrapper<T>::Type;
};

B<A> g_b;
'''

@conf
def check_friend_typename(self):
    if self.check_cxx(msg='Checking for friend typename-specifier',
                      fragment=FRIEND_TYPENAME,
                      features='cxx', mandatory=False):
        self.define('HAVE_CXX_FRIEND_TYPENAME', 1)
    elif self.check_cxx(msg='Checking for friend typename using wrapper',
                      fragment=FRIEND_WRAPPER,
                      features='cxx', mandatory=True):
        self.define('HAVE_CXX_FRIEND_TYPENAME_WRAPPER', 1)

OVERRIDE = '''
class Base
{
  virtual void
  f(int a);
};

class Derived : public Base
{
  virtual void
  f(int a) override;
};

class Final : public Derived
{
  virtual void
  f(int a) final;
};
'''

@conf
def check_override(self):
    if self.check_cxx(msg='Checking for override and final specifiers',
                      fragment=OVERRIDE,
                      features='cxx', mandatory=False):
        self.define('HAVE_CXX_OVERRIDE_FINAL', 1)

STD_TO_STRING = '''
#include <string>
int
main()
{
  std::string s = std::to_string(0);
  s = std::to_string(0l);
  s = std::to_string(0ll);
  s = std::to_string(0u);
  s = std::to_string(0ul);
  s = std::to_string(0ull);
  s = std::to_string(0.0f);
  s = std::to_string(0.0);
  s = std::to_string(0.0l);
  s.clear();
}
'''

@conf
def check_std_to_string(self):
    if self.check_cxx(msg='Checking for std::to_string',
                      fragment=STD_TO_STRING,
                      features='cxx', mandatory=False):
        self.define('HAVE_STD_TO_STRING', 1)

VECTOR_INSERT_ERASE_CONST_ITERATOR = '''
#include <vector>
int
main()
{
  std::vector<int> v;
  std::vector<int>::const_iterator it = v.cbegin();

  v.insert(it, 2);
  it = v.cend() - 1;
  v.erase(it);
}
'''

@conf
def check_vector_const_iterators(self):
    if self.check_cxx(msg='Checking for std::vector::insert with const_iterator',
                      fragment=VECTOR_INSERT_ERASE_CONST_ITERATOR,
                      features='cxx', mandatory=False):
        self.define('HAVE_VECTOR_INSERT_ERASE_CONST_ITERATOR', 1)

def configure(conf):
    conf.check_friend_typename()
    conf.check_override()
    conf.check_std_to_string()
    conf.check_vector_const_iterators()
