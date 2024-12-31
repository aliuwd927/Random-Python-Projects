import requests
from bs4 import BeautifulSoup

url = "https://www.youtube.com/@GxAce/videos"
url2 = "https://realpython.github.io/fake-jobs/"
response = requests.get(url)

soup = BeautifulSoup(response.text, 'html.parser')

upload_date =  soup.find_all(id="script")


print(upload_date)