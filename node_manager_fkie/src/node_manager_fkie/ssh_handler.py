# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
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

import os
import sys
import shlex
import subprocess
import threading

try:
  import paramiko
except Exception, e:
  print  >> sys.stderr, e
  sys.exit(1)

import rospy
import node_manager_fkie as nm
from supervised_popen import SupervisedPopen

class AuthenticationRequest(Exception):
  ''' '''
  
  def __init__(self, user, host, error):
    Exception.__init__(self)
    self.user = user
    self.host = host
    self.error = error
  
  def __str__(self):
    return "AuthenticationRequest on "  + self.host + " for " + self.user + "::" + repr(self.error)


class SSHhandler(object):
  '''
  The class to handle the SSH sessions to the remote hosts.
  '''
  SSH_SESSIONS = {} # host :session
  SSH_AUTH = {} # host : user


  def __init__(self):
    self.mutex = threading.RLock()

  def close(self):
    '''
    Closes all open SSH sessions. Used on the closing the node manager.
    '''
    # close all ssh sessions
    for ssh in SSHhandler.SSH_SESSIONS.keys():
      s = SSHhandler.SSH_SESSIONS.pop(ssh)
      if not s._transport is None:
        s.close()
      del s

  def transfer(self, host, local_file, remote_file, user=None, pw=None, auto_pw_request=False):
    '''
    Copies a file to a remote location using paramiko sfpt.
    @param host: the host
    @type host: C{str}
    @param local_file: the local file
    @type local_file: str
    @param remote_file: the remote file name
    @type remote_file: str
    @param user: user name
    @param pw: the password
    '''
    with self.mutex:
      try:
        ssh = self._getSSH(host, nm.settings().host_user(host) if user is None else user, pw, True, auto_pw_request)
        if not ssh is None:
          sftp = ssh.open_sftp()
          try:
            sftp.mkdir(os.path.dirname(remote_file))
          except:
            pass
          sftp.put(local_file, remote_file)
          rospy.loginfo("SSH COPY %s -> %s@%s:%s", local_file, ssh._transport.get_username(), host, remote_file)
      except AuthenticationRequest as e:
        raise
      except Exception, e:
        raise

  def ssh_exec(self, host, cmd, user=None, pw=None, auto_pw_request=False, get_pty=False, close_stdin=False, close_stdout=False, close_stderr=False):
    '''
    Executes a command on remote host. Returns the output channels with 
    execution result or None. The connection will be established using paramiko 
    SSH library.
    @param host: the host
    @type host: C{str}
    @param cmd: the list with command and arguments
    @type cmd: C{[str,...]}
    @param user: user name
    @param pw: the password
    @return: the 4-tuple stdin, stdout, stderr and boolean of the executing command
    @rtype: C{tuple(ChannelFile, ChannelFile, ChannelFile, boolean)}
    @see: U{http://www.lag.net/paramiko/docs/paramiko.SSHClient-class.html#exec_command}
    '''
    with self.mutex:
      try:
        ssh = self._getSSH(host, nm.settings().host_user(host) if user is None else user, pw, True, auto_pw_request)
        if not ssh is None:
          cmd_str = str(' '.join(cmd))
          rospy.loginfo("REMOTE execute on %s@%s: %s", ssh._transport.get_username(), host, cmd_str)
          (stdin, stdout, stderr) = (None, None, None)
          if get_pty:
            (stdin, stdout, stderr) = ssh.exec_command(cmd_str, get_pty=get_pty)
          else:
            (stdin, stdout, stderr) = ssh.exec_command(cmd_str)
          if close_stdin:
            stdin.close()
          if close_stdout:
            stdout.close()
          if close_stderr:
            stderr.close()
          return stdin, stdout, stderr, True
      except AuthenticationRequest as e:
        raise
      except Exception as e:
        raise
    raise Exception('Can not login @%s'%host)


  def ssh_x11_exec(self, host, cmd, title=None, user=None):
    '''
    Executes a command on remote host using a terminal with X11 forwarding. 
    @todo: establish connection using paramiko SSH library.
    @param host: the host
    @type host: C{str}
    @param cmd: the list with command and arguments
    @type cmd: C{[str,...]}
    @param title: the title of the new opened terminal, if it is None, no new terminal will be created
    @type title: C{str} or C{None}
    @param user: user name
    @return: the result of C{subprocess.Popen(command)} 
    @see: U{http://docs.python.org/library/subprocess.html?highlight=subproces#subprocess}
    '''
    with self.mutex:
      try:
        # workaround: use ssh in a terminal with X11 forward
        user = nm.settings().host_user(host) if user is None else user
        if self.SSH_AUTH.has_key(host):
          user = self.SSH_AUTH[host]
        # generate string for SSH command
        ssh_str = ' '.join(['/usr/bin/ssh',
                            '-aqtx',
                            '-oClearAllForwardings=yes',
                            '-oConnectTimeout=5',
                            '-oStrictHostKeyChecking=no',
                            '-oVerifyHostKeyDNS=no',
                            '-oCheckHostIP=no',
                            ''.join([user, '@', host])])
        if not title is None:
          cmd_str = nm.settings().terminal_cmd([ssh_str, ' '.join(cmd)], title)
        else:
          cmd_str = str(' '.join([ssh_str, ' '.join(cmd)]))
        rospy.loginfo("REMOTE x11 execute on %s: %s", host, cmd_str)
        return SupervisedPopen(shlex.split(cmd_str), id=str(title), description="REMOTE x11 execute on %s: %s"%(host, cmd_str))
      except:
        raise

  def _getSSH(self, host, user, pw=None, do_connect=True, auto_pw_request=False):
    '''
    @return: the paramiko ssh client
    @rtype: L{paramiko.SSHClient} 
    @raise BadHostKeyException: - if the server's host key could not be verified
    @raise AuthenticationException: - if authentication failed
    @raise SSHException: - if there was any other error connecting or establishing an SSH session
    @raise socket.error: - if a socket error occurred while connecting
    '''
    session = SSHhandler.SSH_SESSIONS.get(host, paramiko.SSHClient())
    if session is None or (not session.get_transport() is None and (not session.get_transport().is_active() or session._transport.get_username() != user)):
      t = SSHhandler.SSH_SESSIONS.pop(host)
      del t
      if self.SSH_AUTH.has_key(host):
        del self.SSH_AUTH[host]
      session = SSHhandler.SSH_SESSIONS.get(host, paramiko.SSHClient())
    if session._transport is None:
      session.set_missing_host_key_policy(paramiko.AutoAddPolicy())
      while (session.get_transport() is None or not session.get_transport().authenticated) and do_connect:
        try:
          session.connect(host, username=user, password=pw, timeout=3, compress=True)
          self.SSH_AUTH[host] = user
        except Exception, e:
#          import traceback
#          print traceback.format_exc(1)
          if str(e) in ['Authentication failed.', 'No authentication methods available', 'Private key file is encrypted']:
            if auto_pw_request:
              #'print "REQUEST PW-AUTO"
              res, user, pw = self._requestPW(user, host)
              if not res:
                return None
              self.SSH_AUTH[host] = user
            else:
              raise AuthenticationRequest(user, host, str(e))
          else:
            rospy.logwarn("ssh connection to %s failed: %s", host, str(e))
            raise Exception(' '.join(["ssh connection to", host, "failed:", str(e)]))
        else:
          SSHhandler.SSH_SESSIONS[host] = session
      if not session.get_transport() is None:
        session.get_transport().set_keepalive(10)
    return session

  def _requestPW(self, user, host):
    '''
    Open the dialog to input the user name and password to open an SSH connection. 
    '''
    from python_qt_binding import QtCore
    from python_qt_binding import loadUi
    from python_qt_binding import QtGui
    import os
    result = False
    pw = None
    pwInput = QtGui.QDialog()
    ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'PasswordInput.ui')
    loadUi(ui_file, pwInput)
    pwInput.setWindowTitle(''.join(['Enter the password for user ', user, ' on ', host]))
    pwInput.userLine.setText(str(user))
    pwInput.pwLine.setText("")
    pwInput.pwLine.setFocus(QtCore.Qt.OtherFocusReason)
    if pwInput.exec_():
      result = True
      user = pwInput.userLine.text()
      pw = pwInput.pwLine.text()
    return result, user, pw
