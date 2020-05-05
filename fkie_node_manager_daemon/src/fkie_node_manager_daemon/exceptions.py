# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Fraunhofer FKIE/CMS, Alexander Tiderko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Fraunhofer nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.



from .common import utf8


class ListSelectionRequest(Exception):
    ''' '''

    def __init__(self, choices, error):
        Exception.__init__(self)
        self.choices = choices
        self.error = error

    def __repr__(self):
        return "%s <choices=%s>::%s" % (self.__class__, utf8(self.choices), repr(self.error))

    def __str__(self):
        return self.error


class BinarySelectionRequest(ListSelectionRequest):
    pass


class LaunchSelectionRequest(ListSelectionRequest):
    pass


class ParamSelectionRequest(ListSelectionRequest):
    pass


class StartException(Exception):
    pass


class AlreadyOpenException(Exception):

    def __init__(self, path, error):
        Exception.__init__(self)
        self.path = path
        self.error = error

    def __repr__(self):
        return "%s <path=%s>::%s" % (self.__class__, utf8(self.path), repr(self.error))

    def __str__(self):
        return self.error


class ResourceNotFound(AlreadyOpenException):
    pass


class RemoteException(Exception):

    def __init__(self, code, error):
        Exception.__init__(self)
        self.code = code
        self.error = error

    def __repr__(self):
        return "%s <code=%s>::%s" % (self.__class__, self.code, repr(self.error))

    def __str__(self):
        return self.error


class ConnectionException(Exception):

    def __init__(self, remote, error):
        Exception.__init__(self)
        self.remote = remote
        self.error = error

    def __repr__(self):
        return "%s %s::%s" % (self.__class__, self.remote, repr(self.error))

    def __str__(self):
        return self.error


class GrpcTimeout(Exception):

    def __init__(self, remote, error):
        Exception.__init__(self)
        self.remote = remote
        self.error = error

    def __repr__(self):
        return "%s <%s>::%s" % (self.__class__, utf8(self.remote), repr(self.error))

    def __str__(self):
        return self.error
