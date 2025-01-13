from importlib.metadata import version, PackageNotFoundError
from platform import python_version

def check_python_version():
    print('Your python version is ', python_version())
    assert python_version()[:3] == '3.9', 'Make sure you use python version == 3.9'

def check_env_setup():
    dependencies = open("./requirements.txt").readlines()
    for line in dependencies:
        pkg_info = line.split("==")
        if len(pkg_info) < 2 or pkg_info[0][0] == "#":
            continue
        try:
            inst_pkg_ver = version(pkg_info[0])
            pkg_ver = str.rstrip(pkg_info[1])
            if inst_pkg_ver == pkg_ver:
                print("Package: " + pkg_info[0] + "==" + pkg_ver + " ✅ ALL GOOD")
            else:
                print("Package mismatch: " + pkg_info[0] + "==" + pkg_ver + " found version = " + inst_pkg_ver)
        except PackageNotFoundError as pnf:
            print("⚠️ The required library below was not found. ")
            print(pkg_info[0] + "==" + pkg_ver)
        except Exception as e:
            print(e)
            print("⚠️ Most often means that you need to remove and reinstall the required library")

if __name__ == '__main__':
    check_python_version()
    check_env_setup()