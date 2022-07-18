# Copyright 2022 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import requests
import tarfile
import os

dir = os.listdir('NatNetSDK/')

if 'lib' in dir and 'include' in dir:
    pass

else:
    #print('Downloading started')

    """ Tiny URL for
      https://s3.amazonaws.com/naturalpoint/software/
      NatNetSDKLinux/ubuntu/NatNet_SDK_4.0_ubuntu.tar
    """
    url = 'https://tinyurl.com/4j3j8434' 
    req = requests.get(url)
    #print(url)
    filename = 'NatNetSDK.tar'

    with open(filename, 'wb') as output_file:
        output_file.write(req.content)
    #print('Downloading completed')

    #print('Extracting started')
    tar = tarfile.open(filename)
    tar.extractall(path='NatNetSDK/')
    tar.close()
    #print('Extracting completed')

    os.remove(filename)
    #print('Removed extra files')
