#!/usr/bin/python3
import telebot
import subprocess

BOT_TOKEN = '8349428815:AAEyYa0CxtoLCV_aVKG_bUnqe6FUMPipQj0'

bot = telebot.TeleBot(BOT_TOKEN)

@bot.message_handler(commands=['start', 'hello'])
def send_welcome(message):
    try:
        msg = subprocess.check_output("/bin/hostname -I", shell=True).decode('ascii').strip()
    except subprocess.CalledProcessError:
        msg = "No se pudo obtener la IP en este momento."
    bot.reply_to(message, msg)

bot.infinity_polling()


