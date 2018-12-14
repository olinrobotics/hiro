import requests, errno, shutil, os, sys, json

def get_page(page_url):
    try:
        res = requests.get(page_url)
    except (requests.exceptions.MissingSchema, requests.exceptions.InvalidSchema) as e:
        print('<Invalid Syntax>')
        return None

    try:
        res.raise_for_status()
    except requests.exceptions.HTTPError:
        return None
        
    return res

# Taken from https://stackoverflow.com/questions/23793987/write-file-to-a-directory-that-doesnt-exist
def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise

def safe_open_wb(path):
    ''' Open "path" for writing, creating any parent directories as needed.
    '''
    mkdir_p(os.path.dirname(path))
    return open(path, 'wb')

# Download image to initially nonexistant directory
def download_img(urls, name):
    for count, url in enumerate(urls):
        res = requests.get(url, stream=True)
        with safe_open_wb('images/' + name + '/' + name + '_' + str(count) + '.jpg') as img:
            shutil.copyfileobj(res.raw, img)

# Download images from google custom search
def fetch_image(name):
    # Fetch images from custom goolge search
    search_url = 'https://www.googleapis.com/customsearch/v1?key=AIzaSyCdKWliVigMt35I7pLM2zqKHftpaFxdCR0&cx=016561402344353211294:au-xlsd-kvu&q=' + name + '&searchType=image&imgSize=large&filetype=jpg'
    search = get_page(search_url)

    # Manually input url if google image url not available
    while not search:
        print("Can't get search url. Enter manually (Type exit to quit):")
        search_url = input()

        if search_url.lower() == 'exit':
            sys.exit()

        search = get_page(search_url)

    if (search.status_code != 200):
        print('Error occured when attempting to fetch images')
    else:
        content = json.loads(search.content)
        # Get urls of images
        items = content['items']
        urls = list(map(lambda item: item['link'], items))
        print(urls)

        # Download images
        download_img(urls, name)