module mm_1 #(
    parameter STATE_WIDTH   = 10,  // 状�?�位�?
    parameter LITERAL_WIDTH = 7,   // 字符位宽
    parameter DATA_WIDTH = 16      // 数据宽度默认16
)(
    // 时钟和复位位
    input  wire                     clk,     // 系统时钟
    input  wire                     rst_n,   // 低电平复位信�?
    
    // 输入接口 - 来自scan模块
    input wire [7:0]                length_from_scan,       // 指针长度
    input wire                      is_pointer_to_mm,       // 是否为指针信�?
    input wire [LITERAL_WIDTH-1:0]  literal_from_scan,      // 字符数据
    input wire [STATE_WIDTH-1:0]    state_from_scan,        // 状�?�数�?
    input wire [14:0]               last_state_index,       // �?后状态索�?

    // 输入接口 - 来自verification模块
    input wire [15 + 24 - 1:0]      ptr_from_v,             // 指针信息(l_index + pointer)
    input wire                      v_ptr_valid,            // 指针有效信号
    input wire [2*DATA_WIDTH*STATE_WIDTH-1:0] states_from_v,    // 来自verification的状态数�?(输出)
    input wire [2*DATA_WIDTH*LITERAL_WIDTH-1:0] literals_from_v, // 来自verification的字符数�?(输出)
    input wire                      v_success,              // 校验成功信号
    input wire                      v_continue,             // �?要继续校验信�?
    input wire                      v_first,                // 是否为第�?次校�?

    // 输入接口 - 来自tracker模块
    input wire [14:0]               tracker_addr,           // tracker的地�?
    input wire                      tracker_addr_valid,     // tracker地址有效信号
    
    // 输出接口 - 到verification模块
    output reg [2*DATA_WIDTH*LITERAL_WIDTH-1:0] v_literals_cache, // 校验用的字符数据
    output reg [2*DATA_WIDTH*STATE_WIDTH-1:0]   v_states_cache,    // 校验用的状�?�数�?
    output reg                                  v_enable,          // 校验使能信号

    // 输出接口 - 到tracker模块
    output reg [2*DATA_WIDTH*STATE_WIDTH-1:0]  states_to_tracker, // 到tracker的状态数�?

    // 输出接口 - 到scan模块
    output reg [STATE_WIDTH-1:0]    last_state             // �?后状�?
);

    // =============================================
    // 状�?�定�?
    // =============================================
    /* 有限状�?�机状�?�定�? */
    localparam 
        IDLE               = 4'd0,   // 初始空闲状�??
        V_CALCUATE_ADDRESS = 4'd1,   // 计算源地�?
        WAIT_FULL_BRAM     = 4'd2,   // 等待BRAM完全写入
        FULL_BRAM          = 4'd3,   // BRAM已完全写入准备读�?
        WAIT               = 4'd4,   // 等待状�??(过渡)
        RECIEVE_DATA       = 4'd5,   // 从RAM接收数据
        COPY_DATA          = 4'd6,   // 复制数据到校验模�?
        C_CALCUATE_ADDRESS = 4'd7,   // 计算校验目标地址
        C_WAIT_FULL_BRAM   = 4'd8,   // 等待BRAM完全写入(用于校验)
        C_FULL_BRAM        = 4'd9,   // BRAM已完全写入准备读�?(用于校验)
        C_WAIT             = 4'd10,  // 等待状�??(过渡)(用于校验)
        C_RECIEVE_DATA     = 4'd11,  // 从RAM接收数据(用于校验)
        C_COPY_DATA        = 4'd12;  // 复制数据到校验模�?(用于校验)

    // =============================================
    // BRAM相关信号
    // =============================================
    /* BRAM读取数据 */
    wire [DATA_WIDTH*STATE_WIDTH-1:0]         states_from_bram0;  // 从RAM0读取的状态数�?
    wire [DATA_WIDTH*STATE_WIDTH-1:0]         states_from_bram1;  // 从RAM1读取的状态数�?
    wire [DATA_WIDTH*STATE_WIDTH-1:0]         cp_states_from_bram0;  // 从RAM0读取的状态数�?
    wire [DATA_WIDTH*STATE_WIDTH-1:0]         cp_states_from_bram1;  // 从RAM1读取的状态数�?
    wire [DATA_WIDTH*LITERAL_WIDTH-1:0]       literals_from_bram0; // 从RAM0读取的字符数�?
    wire [DATA_WIDTH*LITERAL_WIDTH-1:0]       literals_from_bram1; // 从RAM1读取的字符数�?

    wire [DATA_WIDTH*STATE_WIDTH-1:0]         s_states_from_bram0;  // 从RAM0读取的状态数�?(douta)
    wire [DATA_WIDTH*STATE_WIDTH-1:0]         s_states_from_bram1;  // 从RAM1读取的状态数�?(douta)
    wire [DATA_WIDTH*STATE_WIDTH-1:0]         s_cp_states_from_bram0;  // 从RAM0读取的状态数�?(douta)
    wire [DATA_WIDTH*STATE_WIDTH-1:0]         s_cp_states_from_bram1;  // 从RAM1读取的状态数�?(douta)
    wire [DATA_WIDTH*LITERAL_WIDTH-1:0]       s_literals_from_bram0; // 从RAM0读取的字符数�?(douta)
    wire [DATA_WIDTH*LITERAL_WIDTH-1:0]       s_literals_from_bram1; // 从RAM1读取的字符数�?(douta)

    /* BRAM写入数据 */
    // scan模块写入BRAM的数�?
    reg [DATA_WIDTH*LITERAL_WIDTH-1:0]     s_literals_to_bram0; // 写入BRAM0的字符数�?(scan)
    reg [DATA_WIDTH*LITERAL_WIDTH-1:0]     s_literals_to_bram1; // 写入BRAM1的字符数�?(scan)
    reg [DATA_WIDTH*STATE_WIDTH-1:0]       s_states_to_bram0;   // 写入BRAM0的状态数�?(scan)
    reg [DATA_WIDTH*STATE_WIDTH-1:0]       s_states_to_bram1;   // 写入BRAM1的状态数�?(scan)
    reg [DATA_WIDTH*STATE_WIDTH-1:0]       s_cp_states_to_bram0; // 写入BRAM0目标状�?�的状�?�数�?(scan)
    reg [DATA_WIDTH*STATE_WIDTH-1:0]       s_cp_states_to_bram1; // 写入BRAM1目标状�?�的状�?�数�?(scan)

    // verification模块写入BRAM的数�?
    reg [DATA_WIDTH*LITERAL_WIDTH-1:0]     v_literals_to_bram0; // 写入BRAM0的字符数�?(verification)
    reg [DATA_WIDTH*LITERAL_WIDTH-1:0]     v_literals_to_bram1; // 写入BRAM1的字符数�?(verification)
    reg [DATA_WIDTH*STATE_WIDTH-1:0]       v_states_to_bram0;   // 写入BRAM0的状态数�?(verification)
    reg [DATA_WIDTH*STATE_WIDTH-1:0]       v_states_to_bram1;   // 写入BRAM1的状态数�?(verification)
    reg [DATA_WIDTH*STATE_WIDTH-1:0]       v_cp_states_to_bram0; // 写入BRAM0目标状�?�的状�?�数�?(verification)
    reg [DATA_WIDTH*STATE_WIDTH-1:0]       v_cp_states_to_bram1; // 写入BRAM1目标状�?�的状�?�数�?(verification)

    /* BRAM控制信号 */
    // scan模块BRAM控制信号
    reg                                    s_state_bram0_we;     // BRAM0状�?�写使能(scan)
    reg                                    s_state_bram1_we;     // BRAM1状�?�写使能(scan)
    reg                                    s_state_enable_bram0; // BRAM0状�?�使�?(scan)
    reg                                    s_state_enable_bram1; // BRAM1状�?�使�?(scan)
    reg [9 : 0]                            s_state_bram0_addr;   // BRAM0状�?�地�?(scan)
    reg [9 : 0]                            s_state_bram1_addr;   // BRAM1状�?�地�?(scan)

    reg                                    s_cp_state_bram0_we;  // BRAM0目标状�?�写使能(scan)
    reg                                    s_cp_state_bram1_we;  // BRAM1目标状�?�写使能(scan)
    reg                                    s_cp_state_enable_bram0; // BRAM0目标状�?�使�?(scan)
    reg                                    s_cp_state_enable_bram1; // BRAM1目标状�?�使�?(scan)
    reg [9 : 0]                            s_cp_state_bram0_addr; // BRAM0目标状�?�地�?(scan)
    reg [9 : 0]                            s_cp_state_bram1_addr; // BRAM1目标状�?�地�?(scan)

    reg                                    s_literal_bram0_we;   // BRAM0字符写使�?(scan)
    reg                                    s_literal_bram1_we;   // BRAM1字符写使�?(scan)
    reg                                    s_literal_enable_bram0; // BRAM0字符使能(scan)
    reg                                    s_literal_enable_bram1; // BRAM1字符使能(scan)
    reg [9 : 0]                            s_literal_bram0_addr; // BRAM0字符地址(scan)
    reg [9 : 0]                            s_literal_bram1_addr; // BRAM1字符地址(scan)

    // verification模块BRAM控制信号
    reg                                    v_state_bram0_we;     // BRAM0状�?�写使能(verification)
    reg                                    v_state_bram1_we;     // BRAM1状�?�写使能(verification)
    reg                                    v_state_enable_bram0; // BRAM0状�?�使�?(verification)
    reg                                    v_state_enable_bram1; // BRAM1状�?�使�?(verification)
    reg [9 : 0]                            v_state_bram0_addr;   // BRAM0状�?�地�?(verification)
    reg [9 : 0]                            v_state_bram1_addr;   // BRAM1状�?�地�?(verification)

    reg                                    v_cp_state_bram0_we;  // BRAM0目标状�?�写使能(verification)
    reg                                    v_cp_state_bram1_we;  // BRAM1目标状�?�写使能(verification)
    reg                                    v_cp_state_enable_bram0; // BRAM0目标状�?�使�?(verification)
    reg                                    v_cp_state_enable_bram1; // BRAM1目标状�?�使�?(verification)
    reg [9 : 0]                            v_cp_state_bram0_addr; // BRAM0目标状�?�地�?(verification)
    reg [9 : 0]                            v_cp_state_bram1_addr; // BRAM1目标状�?�地�?(verification)

    reg                                    v_literal_bram0_we;   // BRAM0字符写使�?(verification)
    reg                                    v_literal_bram1_we;   // BRAM1字符写使�?(verification)
    reg                                    v_literal_enable_bram0; // BRAM0字符使能(verification)
    reg                                    v_literal_enable_bram1; // BRAM1字符使能(verification)
    reg [9 : 0]                            v_literal_bram0_addr; // BRAM0字符地址(verification)
    reg [9 : 0]                            v_literal_bram1_addr; // BRAM1字符地址(verification)
    
    // =============================================
    // 数据缓存
    // =============================================
    /* 数据缓存�? */
    reg [2*DATA_WIDTH*LITERAL_WIDTH-1:0]     s_literals_cache;   // scan模块字符数据缓存
    reg [2*DATA_WIDTH*STATE_WIDTH-1:0]       s_states_cache;      // scan模块状�?�数据缓�?
    reg [2*DATA_WIDTH*LITERAL_WIDTH-1:0]     literals_block;      // 从RAM读取的字符数据块(用于填充v_cache)
    reg [2*DATA_WIDTH*STATE_WIDTH-1:0]       states_block;        // 从RAM读取的状态数据块(用于填充v_cache)
    reg [2*DATA_WIDTH*LITERAL_WIDTH-1:0]     tmp_literals_block;  // 字符数据块缓�?(在校验成功后回写)
    reg [2*DATA_WIDTH*STATE_WIDTH-1:0]       tmp_states_block;    // 状�?�数据块缓存(在校验成功后回写)
    reg [2*DATA_WIDTH*LITERAL_WIDTH-1:0]     t_literals_block;  // 字符数据块缓�?(在校验成功后回写)
    reg [2*DATA_WIDTH*STATE_WIDTH-1:0]       t_states_block;    // 状�?�数据块缓存(在校验成功后回写)

    // =============================================
    // 控制变量
    // =============================================
    reg [3:0] current_state, next_state;     // 状�?�机的当前状态和下一状�??
    reg [14:0] s_cache_literal_index;        // scan模块字符数据的索�?
    reg [14:0] s_cache_state_index;          // scan模块状�?�数据的索引
    reg [14:0] copy_addr;                    // 当前操作的源地址
    reg [14:0] write_back_addr;              // 写回地址
    reg [14:0] write_back_last_addr;         // 写回结束地址(index + len)
    reg [1:0] last_state_step;               // 获取�?后状态的分步操作
    reg [1:0] tracker_step;                  // tracker模块状�?�的分步操作
    reg       write_back_step;                // 写回数据的分步操�?(当first�?1�?)
    reg [7:0] write_back_len;                // 写回数据的长�?

    // =============================================
    // 指针解析
    // =============================================
    /* 从verification模块的指针信息中解析出各个字�? */
    wire [14:0] l_index;        // 指针索引
    wire [7:0]  ptr_length;     // 指针长度
    wire [15:0] ptr_distance;   // 指针距离
    
    assign l_index = ptr_from_v[38:24];      // 索引位置
    assign ptr_length = ptr_from_v[23:16];    // 指针长度
    assign ptr_distance = ptr_from_v[15:0];   // 指针距离


    // =============================================
    // scan模块数据处理逻辑
    // =============================================
    /**
     * 处理来自scan模块的数据：
     * 1. 将字符数据和状�?�数据缓存起�?
     * 2. 在�?�当时�?�写入BRAM
     * 3. 处理指针情况
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 复位�?有变�?
            s_literals_cache <= 0;
            s_states_cache <= 0;
            s_cache_literal_index <= 0;
            s_cache_state_index <= 0;
            s_literal_enable_bram0 <= 0;
            s_state_enable_bram0 <= 0;
            s_literal_enable_bram1 <= 0;
            s_state_enable_bram1 <= 0;
        end
        else begin
            if (!is_pointer_to_mm) begin
                // 写入字符数据
                s_cache_literal_index <= s_cache_literal_index + 1;
                // 将字符数据写入缓存的相应位置
                s_literals_cache[((s_cache_literal_index % (2*DATA_WIDTH)) + 1) * LITERAL_WIDTH - 1 -: LITERAL_WIDTH] <= literal_from_scan;
                
                s_cache_state_index <= s_cache_state_index + 1;
                // 将状态数据写入缓存的相应位置
                s_states_cache[((s_cache_state_index % (2*DATA_WIDTH)) + 1) * STATE_WIDTH - 1 -: STATE_WIDTH] <= state_from_scan;
                
                // 在�?�当时�?�刷新写入BRAM
                // 写入到[15]位时，刷新写入BRAM0
                if((s_cache_literal_index % (2*DATA_WIDTH)) == DATA_WIDTH-1) begin 
                    // 写入字符数据到BRAM0
                    s_literal_enable_bram0 <= 1;
                    s_literal_bram0_we <= 1;
                    s_literal_bram0_addr <= s_cache_literal_index[14:5];
                    s_literals_to_bram0 <= {literal_from_scan, s_literals_cache[15 * LITERAL_WIDTH - 1:0]};
                end
                // 状�?�数据写入到BRAM0
                else if((s_cache_state_index % (2*DATA_WIDTH)) == DATA_WIDTH-1) begin
                    // 写入状�?�BRAM0
                    s_state_enable_bram0 <= 1;
                    s_state_bram0_we <= 1;
                    s_state_bram0_addr <= s_cache_state_index[14:5];
                    s_states_to_bram0 <= {state_from_scan, s_states_cache[15 * STATE_WIDTH - 1:0]};
                    
                    // 同时写入目标状�?�BRAM0
                    s_cp_state_enable_bram0 <= 1;
                    s_cp_state_bram0_we <= 1;
                    s_cp_state_bram0_addr <= s_cache_state_index[14:5];
                    s_cp_states_to_bram0 <= {state_from_scan, s_states_cache[15 * STATE_WIDTH - 1:0]};
                end
                
                // 写入到[31]位时，刷新写入BRAM1
                if((s_cache_literal_index % (2*DATA_WIDTH)) == (2*DATA_WIDTH-1)) begin                          
                    s_literal_enable_bram1 <= 1;
                    s_literal_bram1_we <= 1;
                    s_literal_bram1_addr <= s_cache_literal_index[14:5];
                    s_literals_to_bram1 <= {literal_from_scan, s_literals_cache[31 * LITERAL_WIDTH - 1 : 16 * LITERAL_WIDTH]};
                end
                // 状�?�数据写入到BRAM1
                else if((s_cache_state_index % (2*DATA_WIDTH)) == (2*DATA_WIDTH-1)) begin
                    // 写入状�?�BRAM1
                    s_state_enable_bram1 <= 1;
                    s_state_bram1_we <= 1;
                    s_state_bram1_addr <= s_cache_state_index[14:5];
                    s_states_to_bram1 <= {state_from_scan, s_states_cache[31*STATE_WIDTH-1 : 16*STATE_WIDTH]};
                    
                    // 同时写入目标状�?�BRAM1
                    s_cp_state_enable_bram1 <= 1;
                    s_cp_state_bram1_we <= 1;
                    s_cp_state_bram1_addr <= s_cache_state_index[14:5];
                    s_cp_states_to_bram1 <= {state_from_scan, s_states_cache[31*STATE_WIDTH-1 : 16*STATE_WIDTH]};
                end
                else begin
                    // 其他情况关闭BRAM使能
                    s_literal_enable_bram0 <= 0;
                    s_state_enable_bram0 <= 0;
                    s_literal_enable_bram1 <= 0;
                    s_state_enable_bram1 <= 0;
                    s_cp_state_enable_bram0 <= 0;
                    s_cp_state_enable_bram1 <= 0;
                end
            end
            else begin
                // 处理指针情况
                s_cache_literal_index <= s_cache_literal_index + length_from_scan;
                s_states_cache[((s_cache_state_index % (2*DATA_WIDTH)) + 1) * STATE_WIDTH - 1 -: STATE_WIDTH] <= state_from_scan;
                s_cache_state_index <= s_cache_state_index + length_from_scan + 1;
            end
        end
    end

    // =============================================
    // 获取�?后状态�?�辑
    // =============================================
    /**
     * 获取�?后状态�?�辑�?
     * 1. 根据索引从BRAM或缓存中读取
     * 2. 分步操作确保正确读取
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_state_step <= 0;
        end
        else begin
            case(last_state_step)
                2'd0: begin
                    // �?查状态数据是否在BRAM�?
                    if(last_state_index[14:5] != s_cache_state_index[14:5]) begin
                        // 在BRAM中准备读�?
                        if(last_state_index[4] == 0) begin 
                            // 在BRAM0�?
                            s_state_enable_bram0 <= 1;
                            s_state_bram0_we <= 0;
                        end
                        else begin
                            // 在BRAM1�?
                            s_state_enable_bram1 <= 1;
                            s_state_bram1_we <= 0;
                        end
                        last_state_step <= 2'd1;
                    end
                    else begin
                        // 在缓存中
                        last_state_step <= 2'd2;
                    end
                end
                
                2'd1: begin
                    // 等待BRAM读取
                    last_state_step <= 2'd2;
                end
                
                2'd2: begin
                    // 获取�?后状�?
                    if(last_state_index[14:5] != s_cache_state_index[14:5]) begin
                        // 从RAM中读�?
                        if(last_state_index[4] == 0) begin 
                            last_state <= s_states_from_bram0[(last_state_index % 32)*STATE_WIDTH +: STATE_WIDTH];
                            s_state_enable_bram0 <= 0;
                            s_state_bram0_we <= 0;
                        end
                        else begin
                            last_state <= s_states_from_bram1[(last_state_index % 32)*STATE_WIDTH +: STATE_WIDTH];
                            s_state_enable_bram1 <= 0;
                            s_state_bram1_we <= 0;
                        end
                    end
                    else begin
                        // 从缓存中读取
                        last_state <= s_states_cache[(last_state_index % 32)*STATE_WIDTH +: STATE_WIDTH];
                    end
                    last_state_step <= 2'd0;
                end
                
                default: last_state_step <= 2'd0;
            endcase
        end
    end

    // =============================================
    // tracker模块处理逻辑
    // =============================================
    /**
     * 为tracker模块提供状�?�数据：
     * 1. 接收tracker的地�?请求
     * 2. 从RAM中读取相应数�?
     * 3. 组合后提供给tracker模块
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 复位控制信号
            s_cp_state_enable_bram0 <= 0;
            s_cp_state_enable_bram1 <= 0;
            s_cp_state_bram0_we <= 0;
            s_cp_state_bram1_we <= 0;
            v_cp_state_enable_bram0 <= 0;
            v_cp_state_enable_bram1 <= 0;
            v_cp_state_bram0_we <= 0;
            v_cp_state_bram1_we <= 0;

            states_to_tracker <= 0;
            tracker_step <= 0;
        end
        else begin
            case(tracker_step)
                2'd0: begin
                    // 地址有效时准备读�?
                    if(tracker_addr_valid) begin
                        v_cp_state_enable_bram0 <= 1;
                        v_cp_state_enable_bram1 <= 1;

                        v_cp_state_bram0_addr <= tracker_addr[14:5];
                        v_cp_state_bram1_addr <= tracker_addr[14:5];
                        tracker_step <= 2'd1;
                    end
                end
                
                2'd1: begin
                    // 关闭BRAM使能
                    v_cp_state_enable_bram0 <= 0;
                    v_cp_state_enable_bram1 <= 0;
                    tracker_step <= 2'd2;
                end
                
                2'd2: begin
                    // 组合数据后提供给tracker
                    states_to_tracker <= {cp_states_from_bram1, cp_states_from_bram0};
                    tracker_step <= 2'd0;
                end
                
                default: tracker_step <= 2'd0;
            endcase
        end
    end

    // =============================================
    // verification模块数据处理逻辑
    // =============================================
    /**
     * 处理verification模块的数据：
     * 1. 处理校验结果
     * 2. 根据校验结果决定是否回写数据
     * 3. 处理数据回写
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 复位控制信号
            t_literals_block <= 0;
            t_states_block <= 0;
            v_literal_enable_bram0 <= 0;
            v_state_enable_bram0 <= 0;
            v_literal_enable_bram1 <= 0;
            v_state_enable_bram1 <= 0;
            v_literal_bram0_we <= 0;
            v_state_bram0_we <= 0;
            v_literal_bram1_we <= 0;
            v_state_bram1_we <= 0;

            v_literals_cache <= 0;
            v_states_cache <= 0;
            v_enable <= 0;
        end
        else begin
           // 校验成功或需要校验时�?要回写数�?
           if(v_first == 1 && (v_success || v_continue)) begin
                // 第一次校验成功或继续校验时准备回写数�?
                v_literal_enable_bram0 <= 1;
                v_state_enable_bram0 <= 1;
                v_literal_enable_bram1 <= 1;
                v_state_enable_bram1 <= 1;
                v_literal_bram0_we <= 1;
                v_state_bram0_we <= 1;
                v_literal_bram1_we <= 1;
                v_state_bram1_we <= 1;

                v_cp_state_enable_bram0 <= 1;
                v_cp_state_enable_bram1 <= 1;
                v_cp_state_bram0_we <= 1;
                v_cp_state_bram1_we <= 1;

                if(write_back_addr[4] == 0) begin
                    // 同一块写�?
                    v_literal_bram0_addr <= write_back_addr[14:5];
                    v_state_bram0_addr <= write_back_addr[14:5];
                    v_literal_bram1_addr <= write_back_addr[14:5];
                    v_state_bram1_addr <= write_back_addr[14:5];

                    v_cp_state_bram0_addr <= write_back_addr[14:5];
                    v_cp_state_bram1_addr <= write_back_addr[14:5];

                    // 准备回写地址的数�?...
                    case(write_back_step)
                        0: begin
                            case(write_back_len)
                                8'd3: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[3 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[3 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd4: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[4 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[4 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd5: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[5 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[5 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd6: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[6 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[6 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd7: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[7 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[7 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd8: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[8 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[8 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd9: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[9 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[9 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd10: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[10 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[10 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd11: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[11 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[11 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd12: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[12 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[12 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd13: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[13 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[13 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd14: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[14 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[14 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd15: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[15 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[15 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                endcase
                            // t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[write_back_len * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                            // t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[write_back_len * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                            write_back_step <= 1;
                        end
                        1: begin
                            v_states_to_bram0 <= t_states_block[16 * STATE_WIDTH - 1:0];
                            v_states_to_bram1 <= t_states_block[32*STATE_WIDTH-1 : 16*STATE_WIDTH];
                            v_literals_to_bram0 <= t_literals_block[16 * LITERAL_WIDTH - 1:0];
                            v_literals_to_bram1 <= t_literals_block[32*LITERAL_WIDTH-1 : 16*LITERAL_WIDTH];

                            v_cp_states_to_bram0 <= t_states_block[16 * STATE_WIDTH - 1:0];
                            v_cp_states_to_bram1 <= t_states_block[32*STATE_WIDTH-1 : 16*STATE_WIDTH];
                            write_back_step <= 0;
                        end
                    endcase
                end
                else begin
                    // 不同块写�?
                    v_literal_bram1_addr <= write_back_addr[14:5];
                    v_state_bram1_addr <= write_back_addr[14:5];
                    v_literal_bram0_addr <= write_back_addr[14:5] + 1;
                    v_state_bram0_addr <= write_back_addr[14:5] + 1;

                    v_cp_state_bram0_addr <= write_back_addr[14:5] + 1;
                    v_cp_state_bram1_addr <= write_back_addr[14:5];

                    // 准备回写地址的数�?...
                    case(write_back_step)
                        0: begin
                           case(write_back_len)
                                8'd3: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[3 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[3 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd4: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[4 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[4 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd5: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[5 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[5 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd6: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[6 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[6 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd7: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[7 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[7 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd8: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[8 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[8 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd9: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[9 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[9 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd10: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[10 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[10 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd11: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[11 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[11 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd12: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[12 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[12 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd13: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[13 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[13 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd14: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[14 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[14 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                8'd15: begin
                                    t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[15 * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                                    t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[15 * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                                end
                                endcase
                            // t_literals_block <= {tmp_literals_block[32 * LITERAL_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * LITERAL_WIDTH],literals_from_v[write_back_len * LITERAL_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * LITERAL_WIDTH - 1 : 0]};
                            // t_states_block <= {tmp_states_block[32 * STATE_WIDTH - 1 : (32 - write_back_last_addr[4:0]) * STATE_WIDTH],states_from_v[write_back_len * STATE_WIDTH - 1 : 0],tmp_literals_block[write_back_addr[4:0] * STATE_WIDTH - 1 : 0]};
                            write_back_step <= 1;
                        end
                        1: begin
                            v_states_to_bram1 <= t_states_block[16 * STATE_WIDTH - 1:0];
                            v_states_to_bram0 <= t_states_block[32*STATE_WIDTH-1 : 16*STATE_WIDTH];
                            v_literals_to_bram1 <= t_literals_block[16 * LITERAL_WIDTH - 1:0];
                            v_literals_to_bram0 <= t_literals_block[32*LITERAL_WIDTH-1 : 16*LITERAL_WIDTH];

                            v_cp_states_to_bram1 <= t_states_block[16 * STATE_WIDTH - 1:0];
                            v_cp_states_to_bram0 <= t_states_block[32*STATE_WIDTH-1 : 16*STATE_WIDTH];
                            write_back_step <= 0;
                        end
                    endcase
                end
           end
           else if(v_first == 0 && (v_success || v_continue)) begin
                // 非第�?次校验时直接回写数据
                v_literal_enable_bram0 <= 1;
                v_state_enable_bram0 <= 1;
                v_literal_enable_bram1 <= 1;
                v_state_enable_bram1 <= 1;
                v_literal_bram0_we <= 1;
                v_state_bram0_we <= 1;
                v_literal_bram1_we <= 1;
                v_state_bram1_we <= 1;

                v_cp_state_enable_bram0 <= 1;
                v_cp_state_enable_bram1 <= 1;
                v_cp_state_bram0_we <= 1;
                v_cp_state_bram1_we <= 1;
                
                if(write_back_addr[4] == 0) begin
                    // 同一块写�?
                    v_literal_bram0_addr <= write_back_addr[14:5];
                    v_state_bram0_addr <= write_back_addr[14:5];
                    v_literal_bram1_addr <= write_back_addr[14:5];
                    v_state_bram1_addr <= write_back_addr[14:5];

                    v_cp_state_bram0_addr <= write_back_addr[14:5];
                    v_cp_state_bram1_addr <= write_back_addr[14:5];

                    // 准备回写地址的数�?
                    v_states_to_bram0 <= states_from_v[16 * STATE_WIDTH - 1:0];
                    v_states_to_bram1 <= states_from_v[32*STATE_WIDTH-1 : 16*STATE_WIDTH];
                    v_literals_to_bram0 <= literals_from_v[16 * LITERAL_WIDTH - 1:0];
                    v_literals_to_bram1 <= literals_from_v[32*LITERAL_WIDTH-1 : 16*LITERAL_WIDTH];

                    v_cp_states_to_bram0 <= states_from_v[16 * STATE_WIDTH - 1:0];
                    v_cp_states_to_bram1 <= states_from_v[32*STATE_WIDTH-1 : 16*STATE_WIDTH];
                end
                else begin
                    // 不同块写�?
                    v_literal_bram1_addr <= write_back_addr[14:5];
                    v_state_bram1_addr <= write_back_addr[14:5];
                    v_literal_bram0_addr <= write_back_addr[14:5] + 1;
                    v_state_bram0_addr <= write_back_addr[14:5] + 1;

                    v_cp_state_bram0_addr <= write_back_addr[14:5] + 1;
                    v_cp_state_bram1_addr <= write_back_addr[14:5];

                    // 准备回写地址的数�?
                    v_states_to_bram1 <= states_from_v[16 * STATE_WIDTH - 1:0];
                    v_states_to_bram0 <= states_from_v[32*STATE_WIDTH-1 : 16*STATE_WIDTH];
                    v_literals_to_bram1 <= literals_from_v[16 * LITERAL_WIDTH - 1:0];
                    v_literals_to_bram0 <= literals_from_v[32*LITERAL_WIDTH-1 : 16*LITERAL_WIDTH];

                    v_cp_states_to_bram1 <= states_from_v[16 * STATE_WIDTH - 1:0];
                    v_cp_states_to_bram0 <= states_from_v[32*STATE_WIDTH-1 : 16*STATE_WIDTH];
                end 
           end
           else begin
                // 其他情况关闭控制信号
                v_literal_enable_bram0 <= 0;
                v_state_enable_bram0 <= 0;
                v_literal_enable_bram1 <= 0;
                v_state_enable_bram1 <= 0;
                v_literal_bram0_we <= 0;
                v_state_bram0_we <= 0;
                v_literal_bram1_we <= 0;
                v_state_bram1_we <= 0;

                v_cp_state_enable_bram0 <= 0;
                v_cp_state_enable_bram1 <= 0;
                v_cp_state_bram0_we <= 0;
                v_cp_state_bram1_we <= 0;
            end 
        end
    end

     // =============================================
    // 状�?�机
    // =============================================
    /**
     * 状�?�机处理逻辑�?
     * 1. 处理来自各个模块的请�?
     * 2. 协调各个模块的工�?
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 复位状�?�机
            current_state <= IDLE;
        end
        else begin
            // 状�?�转�?
            current_state <= next_state;
            
            case (current_state)
                IDLE: begin
                    // 初始时BRAM使能关闭
                    v_literal_enable_bram0 <= 1;
                    v_state_enable_bram0 <= 1;
                    v_literal_enable_bram1 <= 1;
                    v_state_enable_bram1 <= 1;

                    v_literal_bram0_we <= 0;
                    v_state_bram0_we <= 0;
                    v_literal_bram1_we <= 0;
                    v_state_bram1_we <= 0;
                end

                V_CALCUATE_ADDRESS: begin
                    // 计算源地�?和目标地�?
                    copy_addr <= l_index - ptr_distance;
                    write_back_addr <= l_index;
                    write_back_len <= ptr_length;
                    write_back_last_addr <= l_index + ptr_length;
                end
                
                WAIT_FULL_BRAM: begin
                    // 等待BRAM完全写入
                end
                
                FULL_BRAM: begin
                    // 准备从RAM读取数据
                    if(copy_addr[4] == 0) begin
                        // 同一块读�?
                        v_literal_bram0_addr <= copy_addr[14:5];
                        v_state_bram0_addr <= copy_addr[14:5];
                        v_literal_bram1_addr <= copy_addr[14:5];
                        v_state_bram1_addr <= copy_addr[14:5];
                    end
                    else begin
                        // 不同块读�?
                        v_literal_bram1_addr <= copy_addr[14:5];
                        v_state_bram1_addr <= copy_addr[14:5];
                        v_literal_bram0_addr <= copy_addr[14:5] + 1;
                        v_state_bram0_addr <= copy_addr[14:5] + 1;
                    end
                end
                
                WAIT: begin
                    // 等待状�?�关闭BRAM使能
                    v_literal_enable_bram0 <= 0;
                    v_state_enable_bram0 <= 0;
                    v_literal_enable_bram1 <= 0;
                    v_state_enable_bram1 <= 0;
                end
                
                RECIEVE_DATA: begin
                    // 接收从RAM读取的数�?
                    if(copy_addr[4] == 0) begin //同一�?
                        literals_block <= {literals_from_bram1, literals_from_bram0};
                        states_block <= {states_from_bram1, states_from_bram0};
                    end
                    else begin //不同�?
                        literals_block <= {literals_from_bram0, literals_from_bram1};
                        states_block <= {states_from_bram0, states_from_bram1};
                    end
                end
                
                COPY_DATA: begin
                    // 将数据复制到校验模块
                    case(ptr_length)
                        8'd3: begin
                            v_literals_cache[3*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 3*LITERAL_WIDTH];
                            v_states_cache[3*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 3*STATE_WIDTH];
                        end
                        8'd4: begin
                            v_literals_cache[4*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 4*LITERAL_WIDTH];
                            v_states_cache[4*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 4*STATE_WIDTH];
                        end
                        8'd5: begin
                            v_literals_cache[5*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 5*LITERAL_WIDTH];
                            v_states_cache[5*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 5*STATE_WIDTH];
                        end
                        8'd6: begin
                            v_literals_cache[6*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 6*LITERAL_WIDTH];
                            v_states_cache[6*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 6*STATE_WIDTH];
                        end
                        8'd7: begin
                            v_literals_cache[7*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 7*LITERAL_WIDTH];
                            v_states_cache[7*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 7*STATE_WIDTH];
                        end
                        8'd8: begin
                            v_literals_cache[8*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 8*LITERAL_WIDTH];
                            v_states_cache[8*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 8*STATE_WIDTH];
                        end
                        8'd9: begin
                            v_literals_cache[9*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 9*LITERAL_WIDTH];
                            v_states_cache[9*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 9*STATE_WIDTH];
                        end
                        8'd10: begin
                            v_literals_cache[10*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 10*LITERAL_WIDTH];
                            v_states_cache[10*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 10*STATE_WIDTH];
                        end
                        8'd11: begin
                            v_literals_cache[11*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 11*LITERAL_WIDTH];
                            v_states_cache[11*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 11*STATE_WIDTH];
                        end
                        8'd12: begin
                            v_literals_cache[12*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 12*LITERAL_WIDTH];
                            v_states_cache[12*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 12*STATE_WIDTH];
                        end
                        8'd13: begin
                            v_literals_cache[13*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 13*LITERAL_WIDTH];
                            v_states_cache[13*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 13*STATE_WIDTH]; 
                        end
                        8'd14: begin
                            v_literals_cache[14*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 14*LITERAL_WIDTH];
                            v_states_cache[14*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 14*STATE_WIDTH]; 
                        end
                        8'd15: begin
                            v_literals_cache[15*LITERAL_WIDTH : 0] <=  literals_block[(copy_addr % 32)*LITERAL_WIDTH +: 15*LITERAL_WIDTH];
                            v_states_cache[15*STATE_WIDTH : 0] <= states_block[(copy_addr % 32)*STATE_WIDTH +: 15*STATE_WIDTH];
                        end
                    endcase
                    v_enable <= 1; // 使能校验
                end
                
                C_CALCUATE_ADDRESS: begin
                    // 计算校验目标地址
                    copy_addr <= l_index;
                    write_back_addr <= l_index;
                end
                
                C_WAIT_FULL_BRAM: begin
                    // 等待BRAM完全写入(用于校验)
                end
                
                C_FULL_BRAM: begin
                    // 准备从RAM读取数据(用于校验)
                    v_literal_enable_bram0 <= 1;
                    v_state_enable_bram0 <= 1;
                    v_literal_enable_bram1 <= 1;
                    v_state_enable_bram1 <= 1;

                    v_literal_bram0_we <= 0;
                    v_state_bram0_we <= 0;
                    v_literal_bram1_we <= 0;
                    v_state_bram1_we <= 0;
                    
                    if(copy_addr[4] == 0) begin
                        // 同一块读�?
                        v_literal_bram0_addr <= copy_addr[14:5];
                        v_state_bram0_addr <= copy_addr[14:5];
                        v_literal_bram1_addr <= copy_addr[14:5];
                        v_state_bram1_addr <= copy_addr[14:5];
                    end
                    else begin
                        // 不同块读�?
                        v_literal_bram1_addr <= copy_addr[14:5];
                        v_state_bram1_addr <= copy_addr[14:5];
                        v_literal_bram0_addr <= copy_addr[14:5] + 1;
                        v_state_bram0_addr <= copy_addr[14:5] + 1;
                    end 
                end
                
                C_WAIT: begin
                    // 等待状�??(用于校验)
                    v_literal_enable_bram0 <= 0;
                    v_state_enable_bram0 <= 0;
                    v_literal_enable_bram1 <= 0;
                    v_state_enable_bram1 <= 0;
                end
                
                C_RECIEVE_DATA: begin
                    // 接收从RAM读取的数�?(用于校验)
                    if(copy_addr[4] == 0) begin //同一�?
                        literals_block <= {literals_from_bram1, literals_from_bram0};
                        states_block <= {states_from_bram1, states_from_bram0};
                    end
                    else begin //不同�?
                        literals_block <= {literals_from_bram0, literals_from_bram1};
                        states_block <= {states_from_bram0, states_from_bram1};
                    end
                end
                
                C_COPY_DATA: begin
                    // 复制数据到校验模�?(用于校验)
                    if(v_continue) begin
                        v_states_cache <= states_block;
                        v_literals_cache <= literals_block;
                        v_enable <= 1;
                    end
                    else begin
                        tmp_states_block <= states_block;
                        tmp_literals_block <= literals_block;
                    end
                end    
            endcase
        end
    end

    // =============================================
    // 状�?�机转移逻辑
    // =============================================
    /**
     * 状�?�机转移逻辑�?
     * 根据当前状�?�和输入信号决定下一状�??
     */
    always @(*) begin
        // 默认保持当前状�??
        next_state = current_state;
        
        // 状�?�转移�?�辑
        case (current_state)
            IDLE: begin
                if(v_ptr_valid && v_continue) begin
                    next_state = C_CALCUATE_ADDRESS;
                end
                else if(v_ptr_valid) begin
                    next_state = V_CALCUATE_ADDRESS;
                end
            end

            V_CALCUATE_ADDRESS: begin
                if(((l_index - ptr_distance + ptr_length) / DATA_WIDTH) != (s_cache_literal_index / DATA_WIDTH)) begin
                    next_state = FULL_BRAM;
                end
                else begin
                    next_state = WAIT_FULL_BRAM;
                end
            end
            
            WAIT_FULL_BRAM: begin
                if(((l_index - ptr_distance + ptr_length) / DATA_WIDTH) != (s_cache_literal_index / DATA_WIDTH)) begin
                    next_state = FULL_BRAM;
                end
            end
            
            FULL_BRAM: begin
                next_state = WAIT;
            end
            
            WAIT: begin
                next_state = RECIEVE_DATA;
            end
            
            RECIEVE_DATA: begin
                next_state = COPY_DATA;
            end
            
            COPY_DATA: begin
                next_state = C_CALCUATE_ADDRESS;
            end
            
            C_CALCUATE_ADDRESS: begin
                if(v_continue) begin
                    if((l_index / DATA_WIDTH) != (s_cache_literal_index / DATA_WIDTH)) begin
                        next_state = C_FULL_BRAM;
                    end
                    else begin
                        next_state = C_WAIT_FULL_BRAM;
                    end
                end
                else begin
                    if(((l_index + ptr_length) / DATA_WIDTH) != (s_cache_literal_index / DATA_WIDTH)) begin
                        next_state = C_FULL_BRAM;
                    end
                    else begin
                        next_state = C_WAIT_FULL_BRAM;
                    end
                end
            end
            
            C_WAIT_FULL_BRAM: begin
                if(v_continue) begin
                    if((l_index / DATA_WIDTH) != (s_cache_literal_index / DATA_WIDTH)) begin
                        next_state = C_FULL_BRAM;
                    end
                end
                else begin
                    if(((l_index + ptr_length) / DATA_WIDTH) != (s_cache_literal_index / DATA_WIDTH)) begin
                        next_state = C_FULL_BRAM;
                    end
                end
            end
            
            C_FULL_BRAM: begin
                next_state = C_WAIT;
            end
            
            C_WAIT: begin
                next_state = C_RECIEVE_DATA;
            end
            
            C_RECIEVE_DATA: begin
                next_state = C_COPY_DATA;
            end
            
            C_COPY_DATA: begin
                next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end

    // =============================================
    // BRAM实例�?
    // =============================================
    /* 字符BRAM0实例�? */
    literals_bram lb0 (
      .clka(clk),    // 端口A时钟
      .ena(s_literal_enable_bram0),      // 端口A使能
      .wea(s_literal_bram0_we),      // 端口A写使�?
      .addra(s_literal_bram0_addr),  // 端口A地址
      .dina(s_literals_to_bram0),    // 端口A写入数据
      .douta(s_literals_from_bram0),                      // 端口A读出数据(不使�?)
      .clkb(clk),    // 端口B时钟
      .enb(v_literal_enable_bram0),      // 端口B使能
      .web(v_literal_bram0_we),      // 端口B写使�?
      .addrb(v_literal_bram0_addr),  // 端口B地址
      .dinb(v_literals_to_bram0),    // 端口B写入数据
      .doutb(literals_from_bram0)    // 端口B读出数据
    );

    /* 字符BRAM1实例�? */
    literals_bram lb1 (
      .clka(clk),
      .ena(s_literal_enable_bram1),
      .wea(s_literal_bram1_we),
      .addra(s_literal_bram1_addr),
      .dina(s_literals_to_bram1),
      .douta(s_literals_from_bram1),
      .clkb(clk),
      .enb(v_literal_enable_bram1),
      .web(v_literal_bram1_we),
      .addrb(v_literal_bram1_addr),
      .dinb(v_literals_to_bram1),
      .doutb(literals_from_bram1)
    );

    /* 状�?�BRAM0实例�? */
    states_bram sb0 (
      .clka(clk),
      .ena(s_state_enable_bram0),
      .wea(s_state_bram0_we),
      .addra(s_state_bram0_addr),
      .dina(s_states_to_bram0),
      .douta(s_states_from_bram0),
      .clkb(clk),
      .enb(v_state_enable_bram0),
      .web(v_state_bram0_we),
      .addrb(v_state_bram0_addr),
      .dinb(v_states_to_bram0),
      .doutb(states_from_bram0)
    );

    /* 状�?�BRAM1实例�? */
    states_bram sb1 (
      .clka(clk),
      .ena(s_state_enable_bram1),
      .wea(s_state_bram1_we),
      .addra(s_state_bram1_addr),
      .dina(s_states_to_bram1),
      .douta(s_states_from_bram1),
      .clkb(clk),
      .enb(v_state_enable_bram1),
      .web(v_state_bram1_we),
      .addrb(v_state_bram1_addr),
      .dinb(v_states_to_bram1),
      .doutb(states_from_bram1)
    );

    /* 目标状�?�BRAM0实例�? */
    states_bram sb_cp0 (
      .clka(clk),
      .ena(s_cp_state_enable_bram0),
      .wea(s_cp_state_bram0_we),
      .addra(s_cp_state_bram0_addr),
      .dina(s_cp_states_to_bram0),
      .douta(s_cp_states_from_bram0),
      .clkb(clk),
      .enb(v_cp_state_enable_bram0),
      .web(v_cp_state_bram0_we),
      .addrb(v_cp_state_bram0_addr),
      .dinb(v_cp_states_to_bram0),
      .doutb(cp_states_from_bram0)
    );

    /* 目标状�?�BRAM1实例�? */
    states_bram sb_cp1 (
      .clka(clk),
      .ena(s_cp_state_enable_bram1),
      .wea(s_cp_state_bram1_we),
      .addra(s_cp_state_bram1_addr),
      .dina(s_cp_states_to_bram1),
      .douta(s_cp_states_from_bram1),
      .clkb(clk),
      .enb(v_cp_state_enable_bram1),
      .web(v_cp_state_bram1_we),
      .addrb(v_cp_state_bram1_addr),
      .dinb(v_cp_states_to_bram1),
      .doutb(cp_states_from_bram1)
    );

endmodule