from flask import Flask, render_template, request, redirect, url_for
import pandas as pd
import os
import datetime

app = Flask(__name__)

# File paths
INVENTORY_FILE = os.path.join(os.path.dirname(__file__), '../warehouse_manager/inventory.csv')
ORDERS_FILE = os.path.join(os.path.dirname(__file__), '../warehouse_manager/orders.csv')
LOG_FILE = os.path.join(os.path.dirname(__file__), '../warehouse_manager/logs.csv')

# Load CSV files
def load_csv(file_path):
    if os.path.exists(file_path):
        return pd.read_csv(file_path)
    else:
        return pd.DataFrame()

# Log function
def log_action(order_id, product, shelf, quantity):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_entry = pd.DataFrame([[timestamp, order_id, product, shelf, quantity]], 
                              columns=["Timestamp", "Order ID", "Product", "Shelf", "Quantity Picked"])
    if os.path.exists(LOG_FILE):
        log_entry.to_csv(LOG_FILE, mode='a', header=False, index=False)
    else:
        log_entry.to_csv(LOG_FILE, mode='w', header=True, index=False)

@app.route('/')
def index():
    inventory = load_csv(INVENTORY_FILE)
    orders = load_csv(ORDERS_FILE)
    logs = load_csv(LOG_FILE)
    return render_template('index.html', inventory=inventory.to_dict(orient='records'), 
                           orders=orders.to_dict(orient='records'), logs=logs.to_dict(orient='records'))

@app.route('/add', methods=['POST'])
def add_entry():
    file_type = request.form['file_type']
    if file_type == 'inventory':
        file_path = INVENTORY_FILE
        df = load_csv(file_path)
        product_name = request.form.get("Product")
        new_data = {col: request.form.get(col, '') for col in df.columns}
        
        if product_name in df["Product"].values:
            for col in df.columns:
                if col != "Product":
                    df.loc[df["Product"] == product_name, col] += int(request.form.get(col, 0))
        else:
            df = pd.concat([df, pd.DataFrame([new_data])], ignore_index=True)
        df.to_csv(file_path, index=False)
    
    elif file_type == 'orders':
        file_path = ORDERS_FILE
        df = load_csv(file_path)
        new_order_id = df["Order ID"].max() + 1 if not df.empty else 1
        
        products = request.form.getlist("Product")
        quantities = request.form.getlist("Qty")
        
        for product, quantity in zip(products, quantities):
            new_data = {"Order ID": new_order_id, "Product": product, "Qty": quantity}
            df = pd.concat([df, pd.DataFrame([new_data])], ignore_index=True)
        
        df.to_csv(file_path, index=False)
    
    return redirect(url_for('index'))

@app.route('/edit', methods=['POST'])
def edit_entry():
    file_type = request.form['file_type']
    row_index = int(request.form['row_index'])
    
    if file_type == 'inventory':
        file_path = INVENTORY_FILE
    elif file_type == 'orders':
        file_path = ORDERS_FILE
    else:
        return redirect(url_for('index'))
    
    df = load_csv(file_path)
    for col in df.columns:
        df.at[row_index, col] = request.form.get(col, '')
    df.to_csv(file_path, index=False)
    return redirect(url_for('index'))

@app.route('/delete', methods=['POST'])
def delete_entry():
    file_type = request.form['file_type']
    row_index = int(request.form['row_index'])
    
    if file_type == 'inventory':
        file_path = INVENTORY_FILE
    elif file_type == 'orders':
        file_path = ORDERS_FILE
    else:
        return redirect(url_for('index'))
    
    df = load_csv(file_path)
    df = df.drop(df.index[row_index])
    df.to_csv(file_path, index=False)
    return redirect(url_for('index'))

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')

