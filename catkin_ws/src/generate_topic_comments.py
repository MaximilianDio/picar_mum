import re
import os
import os.path

def find_node_paths():
    file_path_list = []
    for directory_name, directory_names, file_names in os.walk("."):
        for file_name in [f for f in file_names if f.endswith("_node.py")]:
            file_path_list.append(
                dict(node_path=os.path.join(directory_name, file_name), node_name=file_name)
            )

    return file_path_list

def find_corresponding_launch_file(file_paths):
    for index, node in enumerate(file_paths):
        launch_file_basename = os.path.splitext(node["node_name"])[0]
        launch_file_name = launch_file_basename + ".launch"
        for directory_name, directory_names, file_names in os.walk("."):
            for file_name in [f for f in file_names if (f == launch_file_name)]:
                file_paths[index]["launch_path"] = os.path.join(directory_name, file_name)
                file_paths[index]["launch_name"] = launch_file_name
                break

    return file_paths

def find_topics(file_handle):
    file_content_raw = file_handle.read()
    file_content = file_content_raw.replace("\n", "")

    # find subscriptions
    subscriptions = []
    subscription_matches = re.findall("rospy.Subscriber\((.*?)\)", file_content)
    for match in subscription_matches:
        match = match.replace(" ", "")
        match = match.split(",")
        topic_name = match[0]
        topic_type_base = match[1]
        topic_type_match = re.search("from (.+).msg import .*?{}".format(topic_type_base), file_content_raw)
        topic_type_full = topic_type_match.group(1) + "/" + topic_type_base
        subscriptions.append(dict(topic_name=topic_name, topic_type=topic_type_full))

    # find publications
    publications = []
    publication_matches = re.findall("rospy.Publisher\((.*?)\)", file_content)
    for match in publication_matches:
        match = match.replace(" ", "")
        match = match.split(",")
        topic_name = match[0]
        topic_type_base = match[1]
        topic_type_match = re.search("from (.+).msg import .*?{}".format(topic_type_base), file_content_raw)
        topic_type_full = topic_type_match.group(1) + "/" + topic_type_base
        publications.append(dict(topic_name=topic_name, topic_type=topic_type_full))


    return dict(subscriptions=subscriptions, publications=publications)


def write_subscriptions(file_handle, subscriptions):
    file_handle.write("    <!-- Subscriptions -->\n")
    for subscription in subscriptions:
        file_handle.write(
            "    <!-- {}: {} -->\n".format(subscription["topic_name"],
                                           subscription["topic_type"])
        )
    file_handle.write("\n")


def write_publications(file_handle, publications):
    file_handle.write("    <!-- Publications -->\n")
    for publication in publications:
        file_handle.write(
            "    <!-- {}: {} -->\n".format(publication["topic_name"],
                                           publication["topic_type"])
        )
    file_handle.write("\n")


def write_new_launchfile(file_handle_old, file_handle_new, topics):
    in_topic_section = False
    for line in file_handle_old:

        if "<!-- Subscriptions -->" in line or "<!-- Publications -->" in line:
            in_topic_section = True

        if in_topic_section and "</launch>" not in line:
            continue

        if "</launch>" in line:
            write_subscriptions(file_handle_new, topics["subscriptions"])
            write_publications(file_handle_new, topics["publications"])

            file_handle_new.write(line)
            break

        file_handle_new.write(line)


if __name__ == "__main__":
    file_paths = find_node_paths()
    file_paths = find_corresponding_launch_file(file_paths)

    for index, dict_entry in enumerate(file_paths):
        file_handle = open(dict_entry["node_path"], "r")
        print(dict_entry["node_name"])
        topics = find_topics(file_handle)
        file_handle.close()

        if not "launch_path" in dict_entry:
            print("\033[91mAttention! '{}' has no launchfile or launchfile is not compliant with naming convention\n"
                  "Path: {}\033[0m\n".format(dict_entry["node_name"], dict_entry["node_path"]))
            continue

        launchfile = dict_entry["launch_path"]
        new_launchfile = launchfile + ".test"
        file_handle_old = open(launchfile, "r")
        file_handle_new = open(new_launchfile, "w")

        write_new_launchfile(file_handle_old, file_handle_new, topics)

        os.popen("mv {} {}".format(new_launchfile, launchfile))


