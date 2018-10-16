import requests
import csv
import os

def is_downloadable(res):
    """
    Does the url contain a downloadable resource?
    Base code from https://www.codementor.io/aviaryan/downloading-files-from-urls-in-python-77q3bs0un, modified Fall 2017 by L.Zuehsow
    """
    header = res.headers
    content_type = header.get('Content-Type')
    content_length = header.get('Content-Length', None)

    if not 'jpeg' in content_type.lower():
        print('False')
        return False
    elif content_length < 1e4:  # 10 kb approx
        print('False')
        return False
    else:
        print('True')
        return True

def download_from_list(url):
    global count
    try:
        response = requests.get(url,timeout=1)
        if is_downloadable(response):
            count += 1
            address = "dataset/clamp/" + str(count) + ".jpg"
            with open(address,"wb") as out_file:
                out_file.write(response.content)
                del response
    except:
        print('Timeout')

if __name__ == "__main__":
    count = 0
    with open('clamp.csv', 'rb') as csvfile:
        csv_reader =csv.reader(csvfile)
        for row in csvfile:
            download_from_list(row)
    # download_from_list('http://img.epinions.com/images/opti/7a/e5/9210e9c1447ec8e7d55f28ce1549b2bb1-resized200.jpg')
