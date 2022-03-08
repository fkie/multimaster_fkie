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
import roslib
import rospy
import shlex
import signal
import socket
import time
import types
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient


from fkie_master_discovery.common import get_hostname, get_port, masteruri_from_ros
from fkie_node_manager_daemon.common import get_cwd
from fkie_node_manager_daemon import host as nmdhost
from fkie_node_manager_daemon import launcher
from fkie_node_manager_daemon import screen
from fkie_node_manager_daemon import url as nmdurl
from fkie_node_manager_daemon.supervised_popen import SupervisedPopen
from fkie_node_manager_daemon.common import package_name, isstring, utf8

import fkie_node_manager as nm


class StartException(Exception):
    pass


CACHED_PKG_PATH = dict()  # {host : {pkg: path}}


class StartHandler(object):
    '''
    This class contains the methods to run the nodes on local and remote machines
    in a screen terminal.
    '''

    def __init__(self):
        pass

    @classmethod
    def runNodeWithoutConfig(cls, host, package, binary, name, args=[], masteruri=None, use_nmd=True, auto_pw_request=False, user=None, pw=None, path=''):
        '''
        Start a node with using a launch configuration.

        :param str hosturi: the host or ip to run the node
        :param str package: the ROS package containing the binary
        :param str binary: the binary of the node to execute
        :param str name: the ROS name of the node (with name space)
        :param args: the list with arguments passed to the binary
        :type args: [str]
        :param bool use_nmd: start the node using node manager daemon
        :param bool auto_pw_request: opens question dialog directly, use True only if the method is called from the main GUI thread
        :raise Exception: on errors while resolving host
        :see: :meth:`fkie_node_manager.is_local()`
        '''
        # create the name with namespace
        args2 = list(args)
        fullname = roslib.names.ns_join(roslib.names.SEP, name)
        namespace = ''
        for a in args:
            if a.startswith('__ns:='):
                namespace = a.replace('__ns:=', '')
                fullname = roslib.names.ns_join(namespace, name)
        args2.append(''.join(['__name:=', name]))
        # run on local host
        if nm.is_local(host, wait=True):
            if not use_nmd:
                if path:
                    cmd = [path]
                else:
                    try:
                        cmd = roslib.packages.find_node(package, binary)
                    except roslib.packages.ROSPkgException as e:
                        # multiple nodes, invalid package
                        raise StartException(utf8(e))
                    # handle different result types str or array of string
                    if isstring(cmd):
                        cmd = [cmd]
                cmd_type = ''
                if cmd is None or len(cmd) == 0:
                    raise StartException('%s in package [%s] not found!' % (binary, package))
                # compatibility for python scripts installed with catkin_install_python()
                # avoid ask for select a binary
                cmd = cls._remove_src_binary(cmd)
                if len(cmd) > 1:
                    # Open selection for executables
                    err = 'Multiple executables with same name in package [%s] found' % package
                    bsel = nm.BinarySelectionRequest(cmd, err)
                    raise nm.InteractionNeededError(bsel, cls.runNodeWithoutConfig,
                                                              {'host': host,
                                                               'package': package,
                                                               'binary': binary,
                                                               'name': name,
                                                               'args': args,
                                                               'masteruri': masteruri,
                                                               'use_nmd': use_nmd,
                                                               'auto_pw_request': auto_pw_request,
                                                               'user': user,
                                                               'pw': pw,
                                                               'path': path
                                                              })
                else:
                    cmd_type = cmd[0]
            new_env = {}  # dict(os.environ)
            if namespace:
                new_env['ROS_NAMESPACE'] = namespace
            if masteruri is not None:
                cls._prepareROSMaster(masteruri)
                new_env['ROS_MASTER_URI'] = masteruri
                if 'ROS_HOSTNAME' in os.environ:
                    # set ROS_HOSTNAME only if node_manager has also one
                    ros_hostname = nmdhost.get_ros_hostname(masteruri, host)
                    if ros_hostname:
                        new_env['ROS_HOSTNAME'] = ros_hostname
            if use_nmd:
                nm.nmd().launch.start_standalone_node(nmdurl.nmduri(), package, binary, name, namespace, args, new_env, masteruri, host)
            else:
                local_env = dict(os.environ)
                local_env.update(new_env)
                cmd_str = utf8(' '.join([screen.get_cmd(fullname, local_env), cmd_type, ' '.join(args2)]))
                rospy.loginfo("Run without config: %s", fullname if use_nmd else cmd_str)
                SupervisedPopen(shlex.split(cmd_str), env=local_env, object_id="Run without config", description="Run without config [%s]%s" % (utf8(package), utf8(binary)))
        else:
            # run on a remote machine
            startcmd = [nm.settings().start_remote_script,
                        '--package', utf8(package),
                        '--node_type', utf8(binary),
                        '--node_name', utf8(fullname)]
            startcmd[len(startcmd):] = args2
            if masteruri is not None:
                startcmd.append('--masteruri')
                startcmd.append(masteruri)
            rospy.loginfo("Run remote on %s: %s", host, ' '.join(startcmd))
            try:
                error = ''
                _, stdout, stderr, ok = nm.ssh().ssh_exec(host, startcmd, user, pw, auto_pw_request, close_stdin=True)
                if ok:
                    output = stdout.read()
                    error = stderr.read()
                    stdout.close()
                    stderr.close()
                    if error:
                        rospy.logwarn("ERROR while start '%s': %s", name, error)
                        raise StartException("The host '%s' reports:\n%s" % (host, error))
                    if output:
                        if output.find(b"dn't") != -1:
                            rospy.logwarn("Warning while start '%s': %s", name, output)
                        else:
                            rospy.loginfo("STDOUT while start '%s': %s", name, output)
                else:
                    if error:
                        rospy.logwarn("ERROR while start '%s': %s", name, error)
                        raise StartException("The host '%s' reports:\n%s" % (host, error))
            except nm.AuthenticationRequest as e:
                raise nm.InteractionNeededError(e, cls.runNodeWithoutConfig, {'host': host, 'package': package, 'binary': binary, 'name': name, 'args': args, 'masteruri': masteruri, 'use_nmd': use_nmd, 'auto_pw_request': auto_pw_request, 'user': user, 'pw': pw, 'path': path})

    @classmethod
    def _remove_src_binary(cls, cmdlist):
        result = []
        count = 0
        if len(cmdlist) > 1:
            for c in cmdlist:
                if c.find('/src/') == -1:
                    result.append(c)
                    count += 1
        else:
            result = cmdlist
        if count > 1:
            # we have more binaries in src directory
            # aks the user
            result = cmdlist
        return result


    @classmethod
    def _prepareROSMaster(cls, masteruri):
        if masteruri is None:
            masteruri = masteruri_from_ros()
        # start roscore, if needed
        try:
            if not os.path.isdir(screen.LOG_PATH):
                os.makedirs(screen.LOG_PATH)
            socket.setdefaulttimeout(3)
            master = xmlrpcclient.ServerProxy(masteruri)
            master.getUri(rospy.get_name())
            # restart ROSCORE on different masteruri?, not now...
#      master_uri = master.getUri(rospy.get_name())
#      if masteruri != master_uri[2]:
#        # kill the local roscore...
#        raise
        except Exception:
            # run a roscore
            master_host = get_hostname(masteruri)
            if nm.is_local(master_host, True):
                master_port = get_port(masteruri)
                new_env = dict(os.environ)
                new_env['ROS_MASTER_URI'] = masteruri
                ros_hostname = nmdhost.get_ros_hostname(masteruri)
                if ros_hostname:
                    new_env['ROS_HOSTNAME'] = ros_hostname
                cmd_args = '%s roscore --port %d' % (screen.get_cmd('/roscore--%d' % master_port), master_port)
                for n in [1, 2, 3, 4]:
                    try:
                        if n == 1:
                            print("Launch ROS Master in screen  ... %s" % (cmd_args))
                            SupervisedPopen(shlex.split(cmd_args), env=new_env, object_id="ROSCORE", description="Start roscore")
                        elif n == 2:
                            print("ROS Master takes too long for start, wait for next 10 sec ...")
                        elif n == 3:
                            print("A really slow start, wait for last 10 sec ...")
                        # wait for roscore to avoid connection problems while init_node
                        result = -1
                        count = 1
                        while result == -1 and count < 11:
                            try:
                                master = xmlrpcclient.ServerProxy(masteruri)
                                result, _, _ = master.getUri(rospy.get_name())  # _:=uri, msg
                                return
                            except Exception:
                                time.sleep(1)
                                count += 1
                        if n == 4 and count >= 11:
                            raise StartException('Cannot connect to ROS-Master: %s\n--> please run "roscore" manually!' % utf8(masteruri))
                    except Exception as e:
                        raise Exception("Error while call '%s': %s" % (cmd_args, utf8(e)))
            else:
                raise Exception("ROS master '%s' is not reachable" % masteruri)
        finally:
            socket.setdefaulttimeout(None)

    def callService(self, service_uri, service, service_type, service_args=[]):
        '''
        Calls the service and return the response.
        To call the service the ServiceProxy can't be used, because it uses
        environment variables to determine the URI of the running service. In our
        case this service can be running using another ROS master. The changes on the
        environment variables is not thread safe.
        So the source code of the rospy.SerivceProxy (tcpros_service.py) was modified.

        :param str service_uri: the URI of the service
        :param str service: full service name (with name space)
        :param service_type: service class
        :type service_type: ServiceDefinition: service class
        :param service_args: arguments
        :return: the tuple of request and response.
        :rtype: (request object, response object)
        :raise StartException: on error
        :see: rospy.SerivceProxy<http://docs.ros.org/kinetic/api/rospy/html/rospy.impl.tcpros_service.ServiceProxy-class.html>
        '''
        service = str(service)
        rospy.loginfo("Call service %s[%s]: %s, %s", utf8(service), utf8(service_uri), utf8(service_type), utf8(service_args))
        from rospy.core import parse_rosrpc_uri, is_shutdown
#    from rospy.msg import args_kwds_to_message
        from rospy.exceptions import TransportInitError, TransportException
        from rospy.impl.tcpros_base import TCPROSTransport, DEFAULT_BUFF_SIZE  # ,TCPROSTransportProtocol
        from rospy.impl.tcpros_service import TCPROSServiceClient
        from rospy.service import ServiceException
        request = service_type._request_class()
        import genpy
        try:
            now = rospy.get_rostime()
            import std_msgs.msg
            keys = {'now': now, 'auto': std_msgs.msg.Header(stamp=now)}
            genpy.message.fill_message_args(request, service_args, keys)
        except genpy.MessageException as e:
            def argsummary(args):
                if type(args) in [tuple, list]:
                    return '\n'.join([' * %s (type %s)' % (a, type(a).__name__) for a in args])
                else:
                    return ' * %s (type %s)' % (args, type(args).__name__)
            raise StartException("Incompatible arguments to call service:\n%s\nProvided arguments are:\n%s\n\nService arguments are: [%s]" % (e, argsummary(service_args), genpy.message.get_printable_message_args(request)))

#    request = args_kwds_to_message(type._request_class, args, kwds)
        protocol = TCPROSServiceClient(service, service_type, headers={})
        transport = TCPROSTransport(protocol, service)
        # initialize transport
        dest_addr, dest_port = parse_rosrpc_uri(service_uri)
        # connect to service
        transport.buff_size = DEFAULT_BUFF_SIZE
        try:
            transport.connect(dest_addr, dest_port, service_uri, timeout=5)
        except TransportInitError as e:
            # can be a connection or md5sum mismatch
            raise StartException(''.join(["unable to connect to service: ", utf8(e)]))
        transport.send_message(request, 0)
        try:
            responses = transport.receive_once()
            if len(responses) == 0:
                raise StartException("service [%s] returned no response" % service)
            elif len(responses) > 1:
                raise StartException("service [%s] returned multiple responses: %s" % (service, len(responses)))
        except TransportException as e:
            # convert lower-level exception to exposed type
            if is_shutdown():
                raise StartException("node shutdown interrupted service call")
            else:
                raise StartException("transport error completing service call: %s" % (utf8(e)))
        except ServiceException as e:
            raise StartException("Service error: %s" % (utf8(e)))
        finally:
            transport.close()
            transport = None
        return request, responses[0] if len(responses) > 0 else None

    def open_terminal(self, host):
        if nm.is_local(host):
            cmd = nm.settings().terminal_cmd(['cd'], '')
            SupervisedPopen(shlex.split(cmd), object_id="Start local terminal", shell=True)
        else:
            cmd = nm.settings().terminal_cmd(["ssh -XC %s@%s" % (nm.settings().host_user(host), host)], '', noclose=True)
            SupervisedPopen(shlex.split(cmd), object_id="Start Terminal on %s" % host)

    @classmethod
    def get_log_path(cls, host, nodes=[], auto_pw_request=False, user=None, pw=None):
        if nm.is_local(host):
            if len(nodes) == 1:
                return screen.get_logfile(node=nodes[0])
            else:
                return screen.LOG_PATH
        else:
            request = '[]' if len(nodes) != 1 else nodes[0]
            try:
                socket.setdefaulttimeout(3)
                _, stdout, _, ok = nm.ssh().ssh_exec(host, [nm.settings().start_remote_script, '--ros_log_path', request], user, pw, auto_pw_request, close_stdin=True, close_stderr=True)
                if ok:
                    output = stdout.read()
                    stdout.close()
                    return output.strip()
                else:
                    raise StartException(utf8(''.join(['Get log path from "', host, '" failed'])))
            except nm.AuthenticationRequest as e:
                raise nm.InteractionNeededError(e, cls.get_log_path, {'host': host, 'nodes': nodes, 'auto_pw_request': auto_pw_request, 'user': user, 'pw': pw})
            finally:
                socket.setdefaulttimeout(None)

    @classmethod
    def openLog(cls, nodename, host, user=None, only_screen=False, only_roslog=False):
        '''
        Opens the log file associated with the given node in a new terminal.

        :param str nodename: the name of the node (with name space)
        :param str host: the host name or ip where the log file are
        :return: True, if a log file was found
        :rtype: bool
        :raise Exception: on errors while resolving host
        :see: :meth:`fkie_node_manager.is_local()`
        '''
        rospy.loginfo("show log for '%s' on '%s'", utf8(nodename), utf8(host))
        title_opt = 'LOG %s on %s' % (nodename, host)
        if nm.is_local(host):
            found = False
            screenLog = screen.get_logfile(node=nodename)
            if not only_roslog:
                if os.path.isfile(screenLog):
                    cmd = nm.settings().terminal_cmd([nm.settings().log_viewer, screenLog], title_opt)
                    rospy.loginfo("open log: %s", cmd)
                    SupervisedPopen(shlex.split(cmd), object_id="Open log", description="Open log for '%s' on '%s'" % (utf8(nodename), utf8(host)))
                    found = True
            # open roslog file
            roslog = screen.get_ros_logfile(nodename)
            if os.path.isfile(roslog) and (not only_screen or not found):
                title_opt = title_opt.replace('LOG', 'ROSLOG')
                cmd = nm.settings().terminal_cmd([nm.settings().log_viewer, roslog], title_opt)
                rospy.loginfo("open ROS log: %s", cmd)
                SupervisedPopen(shlex.split(cmd), object_id="Open log", description="Open log for '%s' on '%s'" % (utf8(nodename), utf8(host)))
                found = True
            if not found:
                rospy.logwarn('no log files found for %s' % utf8(nodename))
            return found
        else:
            if not only_roslog:
                _ps = nm.ssh().ssh_x11_exec(host, [nm.settings().start_remote_script, '--show_screen_log', nodename], title_opt, user)
            if not only_screen:
                _ps = nm.ssh().ssh_x11_exec(host, [nm.settings().start_remote_script, '--show_ros_log', nodename], title_opt.replace('LOG', 'ROSLOG'), user)
        return False

    @classmethod
    def delete_log(cls, nodename, grpc_uri, auto_pw_request=False, user=None, pw=None):
        '''
        Deletes the log file associated with the given node.

        :param  str nodename: the name of the node (with name space)
        :param str grpc_uri: uri of the node manager daemon where to delete log
        :raise Exception: on errors while resolving host
        :see: :meth:`fkie_node_manager.is_local()`
        '''
        try:
            nm.nmd().screen.delete_log(grpc_uri, [nodename])
        except Exception as err:
            rospy.logwarn("delete log using SSH because of error: %s" % utf8(err))
            host = get_hostname(grpc_uri)
            if nm.is_local(host):
                screenLog = screen.get_logfile(node=nodename)
                pidFile = screen.get_pidfile(node=nodename)
                roslog = screen.get_ros_logfile(nodename)
                if os.path.isfile(screenLog):
                    os.remove(screenLog)
                if os.path.isfile(pidFile):
                    os.remove(pidFile)
                if os.path.isfile(roslog):
                    os.remove(roslog)
            else:
                try:
                    # output ignored: output, error, ok
                    _, stdout, _, ok = nm.ssh().ssh_exec(host, [nm.settings().start_remote_script, '--delete_logs', nodename], user, pw, auto_pw_request, close_stdin=True, close_stdout=False, close_stderr=True)
                    if ok:
                        stdout.readlines()
                        stdout.close()
                except nm.AuthenticationRequest as e:
                    raise nm.InteractionNeededError(e, cls.delete_log, {'nodename': nodename, 'grpc_uri': host, 'auto_pw_request': auto_pw_request, 'user': user, 'pw': pw})

    def kill(self, host, pid, auto_pw_request=False, user=None, pw=None):
        '''
        Kills the process with given process id on given host.

        :param str host: the name or address of the host, where the process must be killed.
        :param int pid: the process id
        :raise StartException: on error
        :raise Exception: on errors while resolving host
        :see: :meth:`fkie_node_manager.is_local()`
        '''
        try:
            self._kill_wo(host, pid, auto_pw_request, user, pw)
        except nm.AuthenticationRequest as e:
            raise nm.InteractionNeededError(e, self.kill, {'host': host, 'pid': pid, 'auto_pw_request': auto_pw_request, 'user': user, 'pw': pw})

    def _kill_wo(self, host, pid, auto_pw_request=False, user=None, pw=None):
        rospy.loginfo("kill %s on %s", utf8(pid), host)
        if nm.is_local(host):
            os.kill(pid, signal.SIGKILL)
            rospy.loginfo("kill: %s", utf8(pid))
        else:
            # kill on a remote machine
            cmd = ['kill -9', str(pid)]
            _, stdout, stderr, ok = nm.ssh().ssh_exec(host, cmd, user, pw, False, close_stdin=True)
            if ok:
                output = stdout.read()
                error = stderr.read()
                stdout.close()
                stderr.close()
                if error:
                    rospy.logwarn("ERROR while kill %s: %s", utf8(pid), error)
                    raise StartException(utf8(''.join(['The host "', host, '" reports:\n', error])))
                if output:
                    rospy.logdebug("STDOUT while kill %s on %s: %s", utf8(pid), host, output)

    def killall_roscore(self, host, auto_pw_request=False, user=None, pw=None):
        '''
        Kills all roscore processes on given host.

        :param str host: the name or address of the host, where the process must be killed.
        :raise StartException: on error
        :raise Exception: on errors while resolving host
        :see: :meth:`fkie_node_manager.is_local()`
        '''
        try:
            self._killall_roscore_wo(host, auto_pw_request, user, pw)
        except nm.AuthenticationRequest as e:
            raise nm.InteractionNeededError(e, self.killall_roscore, {'host': host, 'auto_pw_request': auto_pw_request, 'user': user, 'pw': pw})

    def _killall_roscore_wo(self, host, auto_pw_request=False, user=None, pw=None):
        rospy.loginfo("killall roscore on %s", host)
        cmd = ['killall', 'roscore']
        if nm.is_local(host):
            SupervisedPopen(cmd, object_id="killall roscore", description="killall roscore")
        else:
            # kill on a remote machine
            _, stdout, stderr, ok = nm.ssh().ssh_exec(host, cmd, user, pw, False, close_stdin=True)
            if ok:
                output = stdout.read()
                error = stderr.read()
                stdout.close()
                stderr.close()
                if error:
                    rospy.logwarn("ERROR while killall roscore on %s: %s" % (host, error))
                    raise StartException('The host "%s" reports:\n%s' % (host, error))
                if output:
                    rospy.logdebug("STDOUT while killall roscore on %s: %s" % (host, output))

    def poweroff(self, host, auto_pw_request=False, user=None, pw=None):
        '''
        poweroff given host.

        :param str host: the name or address of the host, where the process must be killed.
        :raise StartException: on error
        :raise Exception: on errors while resolving host
        :see: :meth:`fkie_node_manager.is_local()`
        '''
        try:
            self._poweroff_wo(host, auto_pw_request, user, pw)
        except nm.AuthenticationRequest as e:
            raise nm.InteractionNeededError(e, self.poweroff, {'host': host, 'auto_pw_request': auto_pw_request, 'user': user, 'pw': pw})

    def _poweroff_wo(self, host, auto_pw_request=False, user=None, pw=None):
        if nm.is_local(host):
            rospy.logwarn("shutdown localhost localhost!")
            cmd = nm.settings().terminal_cmd(['sudo poweroff'], "poweroff")
            SupervisedPopen(shlex.split(cmd), object_id="poweroff", description="poweroff")
        else:
            rospy.loginfo("poweroff %s", host)
            # kill on a remote machine
            cmd = ['sudo poweroff']
            _ = nm.ssh().ssh_x11_exec(host, cmd, 'Shutdown %s' % host, user)

    def rosclean(self, grpc_uri, auto_pw_request=False, user=None, pw=None):
        '''
        rosclean purge on given host.
        :param str grpc_uri: the address of the node manager daemon where rosclean is called.
        :raise StartException: on error
        :raise Exception: on errors while resolving host
        :see: L{fkie_node_manager.is_local()}
        '''
        try:
            self._rosclean_wo(grpc_uri, auto_pw_request, user, pw)
        except nm.AuthenticationRequest as e:
            raise nm.InteractionNeededError(e, self.rosclean, {'grpc_uri': grpc_uri, 'auto_pw_request': auto_pw_request, 'user': user, 'pw': pw})

    def _rosclean_wo(self, grpc_uri, auto_pw_request=False, user=None, pw=None):
        try:
            nm.nmd().screen.rosclean(grpc_uri)
        except Exception as err:
            host = get_hostname(grpc_uri)
            if nm.is_local(host):
                rospy.loginfo("rosclean purge on localhost!")
                cmd = nm.settings().terminal_cmd(['rosclean purge -y'], "rosclean")
                SupervisedPopen(shlex.split(cmd), object_id="rosclean", description="rosclean")
            else:
                rospy.logwarn("use SSH to run 'rosclean' because of error: %s" % utf8(err))
                # kill on a remote machine
                cmd = ['rosclean purge -y']
                _ = nm.ssh().ssh_x11_exec(host, cmd, 'rosclean purge on %s' % host, user)

    @classmethod
    def transfer_file_nmd(cls, grpc_url, path, auto_pw_request=False, user=None, pw=None):
        '''
        Copies the given file to the remote host. Uses caching of remote paths.

        :param str grpc_url: destination grpc server
        :param str path: file to transfer
        '''
        try:
            nm.nmd().file.copy(path, grpc_url)
        except Exception as err:
            host = get_hostname(grpc_url)
            _uri, path = nmdurl.split(path)
            rospy.logwarn("use SSH to transfer file '%s' to '%s', because of error: %s" % (path, host, utf8(err)))
            cls.transfer_files(host, path, auto_pw_request, user, pw)

    @classmethod
    def transfer_files(cls, host, path, auto_pw_request=False, user=None, pw=None):
        '''
        Copies the given file to the remote host. Uses caching of remote paths.
        '''
        # get package of the file
        if nm.is_local(host):
            # it's local -> no copy needed
            return
        (pkg_name, pkg_path) = package_name(os.path.dirname(path))
        if pkg_name is not None:
            # get the subpath of the file
            subfile_path = path.replace(pkg_path, '')
            # get the path of the package on the remote machine
            try:
                output = ''
                error = ''
                ok = True
                if host in CACHED_PKG_PATH and pkg_name in CACHED_PKG_PATH[host]:
                    output = CACHED_PKG_PATH[host][pkg_name]
                else:
                    if host not in CACHED_PKG_PATH:
                        CACHED_PKG_PATH[host] = dict()
                    _, stdout, stderr, ok = nm.ssh().ssh_exec(host, [nm.settings().start_remote_script, '--package', pkg_name], user, pw, auto_pw_request, close_stdin=True)
                    output = stdout.read()
                    error = stderr.read()
                    stdout.close()
                    stderr.close()
                if ok:
                    if error:
                        rospy.logwarn("ERROR while transfer %s to %s: %s", path, host, error)
                        raise StartException(utf8(''.join(['The host "', host, '" reports:\n', error])))
                    if output:
                        CACHED_PKG_PATH[host][pkg_name] = output
                        nm.ssh().transfer(host, path, os.path.join(output.strip(), subfile_path.strip(os.sep)), user)
                    else:
                        raise StartException("Remote host no returned any answer. Is there the new version of node_manager installed?")
                else:
                    raise StartException("Can't get path from remote host. Is there the new version of node_manager installed?")
            except nm.AuthenticationRequest as e:
                raise nm.InteractionNeededError(e, cls.transfer_files, {'host': host, 'path': path, 'auto_pw_request': auto_pw_request, 'user': user, 'pw': pw})

    @classmethod
    def ntpdate(cls, host, cmd, user=None, pw=None):
        '''
        Opens the log file associated with the given node in a new terminal.

        :param str host: the host name or ip where the log file are
        :param str cmd: command to set the time
        :return: True, if a log file was found
        :rtype: bool
        :raise Exception: on errors while resolving host
        :see: :meth:`fkie_node_manager.is_local()`
        '''
        mesg = "synchronize time on '%s' using '%s'" % (utf8(host), cmd)
        rospy.loginfo(mesg)
        title_opt = "ntpdate on %s" % str(host)  # '%s on %s' % (cmd, host)
        if nm.is_local(host):
            cmd = nm.settings().terminal_cmd([cmd], title_opt, noclose=True)
            rospy.loginfo("EXEC: %s" % cmd)
            _ps = SupervisedPopen(shlex.split(cmd), object_id=cmd, description=mesg)
        else:
            _ps = nm.ssh().ssh_x11_exec(host, [cmd, ';echo "";echo "this terminal will be closed in 10 sec...";sleep 10'], title_opt, user)
        return False

    @classmethod
    def bc_run_node(cls, name, grpc_path='grpc://localhost:12321', masteruri='', reload_global_param=False, loglevel='', logformat='', auto_pw_request=False, user=None, pw=None):
        '''
        Start the node with given name from the given configuration.

        :param runcfg: the configuration containing the start parameter
        :type runcfg: AdvRunCfg
        :raise StartException: if the screen is not available on host.
        :raise Exception: on errors while resolving host
        :see: :meth:`fkie_node_manager.is_local()`
        '''
        startcfg = nm.nmd().launch.get_start_cfg(name, grpc_path, masteruri, reload_global_param=reload_global_param, loglevel=loglevel, logformat=logformat)
        new_env = dict(startcfg.env)
        # set logging options
        if startcfg.namespace:
            new_env['ROS_NAMESPACE'] = startcfg.namespace
        # set logging
        if startcfg.logformat:
            new_env['ROSCONSOLE_FORMAT'] = '%s' % startcfg.logformat
#         if startcfg.loglevel:
#             new_env['ROSCONSOLE_CONFIG_FILE'] = launcher._rosconsole_cfg_file(startcfg.package, startcfg.loglevel)
        args = []
        # set name and namespace of the node
        if startcfg.name:
            args.append("__name:=%s" % startcfg.name)
        if startcfg.namespace:
            args.append("__ns:=%s" % startcfg.namespace)
        # add remap arguments
        for key, val in startcfg.remaps.items():
            args.append("%s:=%s" % (key, val))
        # handle respawn
        if startcfg.respawn:
            if startcfg.respawn_delay > 0:
                new_env['RESPAWN_DELAY'] = '%d' % startcfg.respawn_delay
            respawn_params = launcher._get_respawn_params(startcfg.fullname, startcfg.params)
            if respawn_params['max'] > 0:
                new_env['RESPAWN_MAX'] = '%d' % respawn_params['max']
            if respawn_params['min_runtime'] > 0:
                new_env['RESPAWN_MIN_RUNTIME'] = '%d' % respawn_params['min_runtime']
        if startcfg.cwd:
            cwd = get_cwd(startcfg.cwd, startcfg.binary_path)
            if cwd:
                args.append('__cwd:=%s' % cwd)
        # check for masteruri
        masteruri = startcfg.masteruri
        on_hostname = startcfg.hostname
        if masteruri is None:
            masteruri = masteruri_from_ros()
        if masteruri is not None:
            new_env['ROS_MASTER_URI'] = masteruri
            if 'ROS_HOSTNAME' in os.environ:
                # set only ROS_HOSTNAME if node manager have also one
                ros_hostname = nmdhost.get_ros_hostname(masteruri, on_hostname)
                if ros_hostname:
                    new_env['ROS_HOSTNAME'] = ros_hostname
            # load params to ROS master
            launcher._load_parameters(masteruri, startcfg.params, startcfg.clear_params)

        abs_paths = list()  # tuples of (parameter name, old value, new value)
        not_found_packages = list()  # package names
#         # set the global parameter
#         if runcfg.masteruri is not None and runcfg.masteruri not in runcfg.roslaunch_config.global_param_done:
#             global_node_names = cls.getGlobalParams(runcfg.roslaunch_config.Roscfg)
#             rospy.loginfo("Register global parameter:\n  %s", '\n  '.join("%s%s" % (utf8(v)[:80], '...' if len(utf8(v)) > 80 else'') for v in global_node_names.values()))
#             abs_paths[len(abs_paths):], not_found_packages[len(not_found_packages):] = cls._load_parameters(runcfg.masteruri, global_node_names, [], runcfg.user, runcfg.pw, runcfg.auto_pw_request)
#             runcfg.roslaunch_config.global_param_done.append(runcfg.masteruri)
#
#         # add params
#         if runcfg.masteruri is not None:
#             nodens = ''.join([n.namespace, n.name, rospy.names.SEP])
#             params = dict()
#             for param, value in runcfg.roslaunch_config.Roscfg.params.items():
#                 if param.startswith(nodens):
#                     params[param] = value
#             clear_params = []
#             for cparam in runcfg.roslaunch_config.Roscfg.clear_params:
#                 if cparam.startswith(nodens):
#                     clear_params.append(cparam)
#             rospy.loginfo("Delete parameter:\n  %s", '\n  '.join(clear_params))
#             rospy.loginfo("Register parameter:\n  %s", '\n  '.join("%s%s" % (utf8(v)[:80], '...' if len(utf8(v)) > 80 else'') for v in params.values()))
#             abs_paths[len(abs_paths):], not_found_packages[len(not_found_packages):] = cls._load_parameters(runcfg.masteruri, params, clear_params, runcfg.user, runcfg.pw, runcfg.auto_pw_request)
        if not nm.is_local(on_hostname, wait=True):
            # start remote
            if not startcfg.package:
                raise StartException("Can't run remote without a valid package name!")
            # setup environment
            env_command = ''
            if new_env:
                try:
                    for k, v in new_env.items():
                        v_value, is_abs_path, found, package = cls._bc_resolve_abs_paths(v, on_hostname, auto_pw_request, user, pw)
                        new_env[k] = v_value
                        if is_abs_path:
                            abs_paths.append(('ENV', "%s=%s" % (k, v), "%s=%s" % (k, v_value)))
                            if not found and package:
                                not_found_packages.append(package)
                    env_command = "env " + ' '.join(["%s=\'%s\'" % (k, v) for (k, v) in new_env.items()])
                except nm.AuthenticationRequest as e:
                    raise nm.InteractionNeededError(e, cls.bc_run_node, {'name': name,
                                                                         'grpc_path': grpc_path,
                                                                         'masteruri': masteruri,
                                                                         'reload_global_param': reload_global_param,
                                                                         'loglevel': loglevel,
                                                                         'logformat': logformat,
                                                                         'auto_pw_request': auto_pw_request,
                                                                         'user': user, 'pw': pw})
            startcmd = [env_command, nm.settings().start_remote_script,
                        '--package', utf8(startcfg.package),
                        '--node_type', utf8(startcfg.binary),
                        '--node_name', utf8(startcfg.fullname),
                        '--node_respawn true' if startcfg.respawn else '']
            if startcfg.masteruri is not None:
                startcmd.append('--masteruri')
                startcmd.append(startcfg.masteruri)
            if startcfg.prefix:
                startcmd[len(startcmd):] = ['--prefix', startcfg.prefix]
            if startcfg.loglevel:
                startcmd.append('--loglevel')
                startcmd.append(startcfg.loglevel)

            # rename the absolute paths in the args of the node
            node_args = []
            error = ''
            output = ''
            try:
                for a in startcfg.args:
                    a_value, is_abs_path, found, package = cls._bc_resolve_abs_paths(a, on_hostname, auto_pw_request, user, pw)
                    node_args.append(a_value)
                    if is_abs_path:
                        abs_paths.append(('ARGS', a, a_value))
                        if not found and package:
                            not_found_packages.append(package)

                startcmd[len(startcmd):] = node_args
                startcmd[len(startcmd):] = args
                rospy.loginfo("Run remote on %s: %s", on_hostname, utf8(' '.join(startcmd)))
                _, stdout, stderr, ok = nm.ssh().ssh_exec(on_hostname, startcmd, user, pw, auto_pw_request, close_stdin=True)
                output = stdout.read()
                error = stderr.read()
                stdout.close()
                stderr.close()
            except nm.AuthenticationRequest as e:
                raise nm.InteractionNeededError(e, cls.bc_run_node,{'name': name,
                                                                    'grpc_path': grpc_path,
                                                                    'masteruri': masteruri,
                                                                    'reload_global_param': reload_global_param,
                                                                    'loglevel': loglevel,
                                                                    'logformat': logformat,
                                                                    'auto_pw_request': auto_pw_request,
                                                                    'user': user, 'pw': pw})
            if ok:
                if error:
                    rospy.logwarn("ERROR while start '%s': %s", startcfg.fullname, error)
                    raise StartException(utf8(''.join(['The host "', on_hostname, '" reports:\n', error])))
                if output:
                    rospy.logdebug("STDOUT while start '%s': %s", startcfg.fullname, output)
            # inform about absolute paths in parameter value
            if len(abs_paths) > 0:
                rospy.loginfo("Absolute paths found while start:\n%s", utf8('\n'.join([''.join([p, '\n  OLD: ', ov, '\n  NEW: ', nv]) for p, ov, nv in abs_paths])))

            if len(not_found_packages) > 0:
                packages = '\n'.join(not_found_packages)
                raise StartException(utf8('\n'.join(['Some absolute paths are not renamed because following packages are not found on remote host:', packages])))

    @classmethod
    def _bc_resolve_abs_paths(cls, value, host, auto_pw_request, user, pw):
        '''
        Replaces the local absolute path by remote absolute path. Only valid ROS
        package paths are resolved.

        :return: value, is absolute path, remote package found (ignore it on local host or if is not absolute path!), package name (if absolute path and remote package NOT found)
        '''
        if isstring(value) and value.startswith('/') and (os.path.isfile(value) or os.path.isdir(value)):
            if nm.is_local(host):
                return value, True, True, ''
            else:
                path = os.path.dirname(value) if os.path.isfile(value) else value
                package, package_path = package_name(path)
                if package:
                    _, stdout, _, ok = nm.ssh().ssh_exec(host, ['rospack', 'find', package], user, pw, auto_pw_request, close_stdin=True, close_stderr=True)
                    output = stdout.read()
                    stdout.close()
                    if ok:
                        if output:
                            value.replace(package_path, output)
                            return value.replace(package_path, output.strip()), True, True, package
                        else:
                            # package on remote host not found!
                            # TODO add error message
                            #      error = stderr.read()
                            pass
                return value, True, False, ''
        else:
            return value, False, False, ''
